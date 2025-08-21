#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/nodeinfo.h"
#include "proto/options.pb.h"
#include "proto_conversions.h"
#include "sif/dynamiccost.h"
#include "sif/pedestriancost.h"
#include <boost/container/flat_map.hpp>
#include <h3/h3api.h>
#include <memory>
#include <pqxx/pqxx>
#include <unordered_map>
using H3LatLng = LatLng;

#ifdef INLINE_TEST
#include "baldr/location.h"
#include "proto_conversions.h"
#include "test.h"
#include "worker.h"
#include <random>
#endif

namespace valhalla {
namespace sif {

namespace {
constexpr float kMinCrimeFactor = 0.0f;
constexpr float kMaxCrimeFactor = 50.0f;
constexpr float kCrimeFactor = 0.5f; // avoid higher crime areas
constexpr ranged_default_t<float> kCrimeFactorRange{kMinCrimeFactor, kCrimeFactor, kMaxCrimeFactor};

} // namespace

struct CrimeData {
  float crime_rate, total_crime;

  CrimeData(float total_crime = 0.0f) : total_crime(total_crime) {
  }
};

class SafeCost : public DynamicCost {
public:
  SafeCost(const Costing& costing, std::shared_ptr<DynamicCost> base_costing);

  explicit SafeCost(const Costing& costing);
  virtual ~SafeCost() {

  };

  float GetSafetyFactor(const baldr::DirectedEdge* edge, const graph_tile_ptr tile) const;

  bool Allowed(const baldr::DirectedEdge* edge,
               const bool is_dest,
               const EdgeLabel& pred,
               const graph_tile_ptr& tile,
               const baldr::GraphId& edgeid,
               const uint64_t current_time,
               const uint32_t tz_index,
               uint8_t& restriction_idx) const override;

  bool AllowedReverse(const baldr::DirectedEdge* edge,
                      const EdgeLabel& pred,
                      const baldr::DirectedEdge* opp_edge,
                      const graph_tile_ptr& tile,
                      const baldr::GraphId& opp_edgeid,
                      const uint64_t current_time,
                      const uint32_t tz_index,
                      uint8_t& restriction_idx) const override;

  Cost EdgeCost(const baldr::DirectedEdge* edge,
                const baldr::TransitDeparture* departure,
                const uint32_t curr_time) const override;

  Cost EdgeCost(const baldr::DirectedEdge* edge,
                const graph_tile_ptr& tile,
                const baldr::TimeInfo& time_info,
                uint8_t& flow_sources) const override;

  virtual bool AllowMultiPass() const override {
    return base_costing_->AllowMultiPass();
  }
  virtual uint32_t GetMaxTransferDistanceMM() override {
    return base_costing_->GetMaxTransferDistanceMM();
  }
  virtual float GetModeFactor() override {
    return base_costing_->GetModeFactor();
  }
  bool IsClosed(const baldr::DirectedEdge* edge, const graph_tile_ptr& tile) const override {
    return base_costing_->IsClosed(edge, tile);
  }
  Cost TransitionCost(const baldr::DirectedEdge* edge,
                 const baldr::NodeInfo* node,
                 const EdgeLabel& pred) const {
    return base_costing_->TransitionCost(edge, node, pred);
  }
  Cost TransitionCostReverse(const uint32_t idx,
                             const baldr::NodeInfo* node,
                             const baldr::DirectedEdge* pred,
                             const baldr::DirectedEdge* edge,
                             const bool speeds /*has_measured_speed*/,
                             const InternalTurn turn /*internal_turn*/) const override{
    return base_costing_->TransitionCostReverse(idx, node, pred, edge, speeds, turn);
  }

  float AStarCostFactor() const override{
    return base_costing_->AStarCostFactor();
  };

  virtual uint8_t travel_type() const override {
    return base_costing_->travel_type();
  }

  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    return base_costing_->Allowed(edge, tile, disallow_mask);
  }

  virtual Cost BSSCost() const override {
    return base_costing_->BSSCost();
  };

  static void LoadCrimeData(const Costing& costing_options);

protected:
  static uint64_t GetH3(const baldr::DirectedEdge* edge, const graph_tile_ptr tile);
  std::shared_ptr<DynamicCost> base_costing_;
  static boost::container::flat_map<uint64_t, CrimeData> crime_data_;

private:
  float InterpolateSafety(float crime_rate) const;
  float max_safety_multiplier_ = 2.5f;
  float crime_factor_;

  Cost
  MutateCost(const baldr::DirectedEdge* edge, const graph_tile_ptr tile, const Cost base_cost) const;
};

boost::container::flat_map<uint64_t, CrimeData> SafeCost::crime_data_;

SafeCost::SafeCost(const Costing& costing, std::shared_ptr<DynamicCost> base_costing)
    : DynamicCost(costing, base_costing->travel_mode(), base_costing->access_mode()),
      base_costing_(base_costing) {
          crime_factor_ = costing.options().crime_factor();
}

SafeCost::SafeCost(const Costing& costing)
    : DynamicCost(costing, TravelMode::kPedestrian, baldr::kPedestrianAccess) {
  throw std::exception();
}

Cost SafeCost::EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr& tile,
                        const baldr::TimeInfo& time_info,
                        uint8_t& flow_sources) const {

  Cost base_cost = base_costing_->EdgeCost(edge, tile, time_info, flow_sources);

  try{
    return MutateCost(edge, tile, base_cost);
  }catch(const std::exception&){
    LOG_INFO("errored");
  }
  return base_cost;
}

Cost SafeCost::EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::TransitDeparture* departure,
                        const uint32_t curr_time) const {
  throw std::runtime_error("SafeCost::EdgeCost does not support transit edges");
}

Cost SafeCost::MutateCost(const baldr::DirectedEdge* edge,
                          const graph_tile_ptr tile,
                          const Cost base_cost) const {
  float safety_factor = GetSafetyFactor(edge, tile);

  float adjusted_cost = base_cost.cost * (1.0f + (safety_factor - 1.0f));

  return {adjusted_cost, base_cost.secs};
}

float SafeCost::GetSafetyFactor(const baldr::DirectedEdge* edge, const graph_tile_ptr tile) const {
  uint64_t h3 = GetH3(edge, tile);

  auto it = crime_data_.find(h3);
  if (it != crime_data_.end()) {
    return InterpolateSafety(std::min(it->second.crime_rate, max_safety_multiplier_));
  }

  return 1.0f;
}

uint64_t SafeCost::GetH3(const baldr::DirectedEdge* edge, const graph_tile_ptr tile) {
  auto edgeInfo = tile->edgeinfo(edge);
  auto shape = edgeInfo.shape();

  double lat = 0;
  double lon = 0;
  for (auto coord : shape) {
    lat += coord.lat();
    lon += coord.lng();
  }
  lat /= double(shape.size());
  lon /= double(shape.size());
  H3LatLng location;
  location.lat = degsToRads(lat);
  location.lng = degsToRads(lon);
  int resolution = 10;
  H3Index index;
  if (latLngToCell(&location, resolution, &index) != E_SUCCESS) {
    throw;
  }
  return index;
}

float SafeCost::InterpolateSafety(float crime_rate) const {
  float factor = crime_rate;
  factor *= crime_factor_;
  return clamp(factor, 1.0f, max_safety_multiplier_);
}

bool SafeCost::Allowed(const baldr::DirectedEdge* edge,
                       const bool is_dest,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx) const {
  return base_costing_->Allowed(edge, is_dest, pred, tile, edgeid, current_time, tz_index,
                                restriction_idx);
}

bool SafeCost::AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              uint8_t& restriction_idx) const {
  return base_costing_->AllowedReverse(edge, pred, opp_edge, tile, opp_edgeid, current_time, tz_index,
                                       restriction_idx);
}

std::mutex mtx;

struct popWithArea {
  float pop;
  double area;
};

void SafeCost::LoadCrimeData(const Costing& costing_options) {
  std::lock_guard<std::mutex> lock(mtx);
  if (crime_data_.empty()) {
    LOG_INFO("Getting crime data");
    std::string connection_string = costing_options.db_connection_string();
    pqxx::connection conn(connection_string);
    pqxx::work txn(conn);
    pqxx::result result = txn.exec("select h3, population from pop_areas;");
    boost::container::flat_map<uint64_t, popWithArea> popAreas;
    popAreas.reserve(result.size());
    for (const auto& row : result) {
      auto h3 = row[0].as<uint64_t>();
      auto pop = row[1].as<float>();
      double area;
      if (cellAreaM2(h3, &area) != E_SUCCESS)
        throw;
      popWithArea popArea{pop, area};

      popAreas.emplace(std::pair(h3, popArea));
    }
    result = txn.exec(
        "select \n"
        "h3, sum(burglary + personal_theft + weapon_crime + bicycle_theft + damage + robbery + shoplifting + violent + anti_social + drugs + vehicle_crime) as total_crime\n"
        "from crime_areas\n"
        "where \n"
        "date >= date_trunc('month', NOW() AT TIME ZONE 'UTC') - interval '3 months'\n"
        "group by h3;");
    crime_data_.reserve(result.size());
    for (const auto& row : result) {
      auto h3 = row[0].as<uint64_t>();
      int total_crime = row[1].as<int>();
      auto cd = CrimeData(float(total_crime));

      H3Index res7;
      if (cellToParent(h3, 7, &res7) != E_SUCCESS) {
        throw;
      }

      auto popArea = popAreas.find(res7);
      if (popArea != popAreas.end()) {
        double areaH3;
        if (cellAreaM2(h3, &areaH3) != E_SUCCESS)
          throw;
        float normalPop = popArea->second.pop * float(areaH3 / popArea->second.area);
        cd.crime_rate = cd.total_crime / fmax(10.f, normalPop);
      } else {
        cd.crime_rate = cd.total_crime / 10.f;
      }

      crime_data_.emplace(std::make_pair(h3, cd));
    }
    txn.commit();
    LOG_INFO(std::to_string(crime_data_.size()) + " crime entries!");
  }
}

void ParseSafeCostOptions(const rapidjson::Document& doc,
                          const std::string& costing_options_key,
                          Costing* c) {
  c->set_type(Costing::safe);
  c->set_name(Costing_Enum_Name(c->type()));
  auto* co = c->mutable_options();

  rapidjson::Value dummy;
  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  JSON_PBF_RANGED_DEFAULT(co, kCrimeFactorRange, json, "/crime_factor", crime_factor);
}

cost_ptr_t CreateSafeCost(const Costing& costing_options) {
  SafeCost::LoadCrimeData(costing_options);
  return std::make_shared<SafeCost>(costing_options, CreatePedestrianCost(costing_options));
}
} // namespace sif
} // namespace valhalla

#ifdef INLINE_TEST
using namespace valhalla;
using namespace sif;
namespace {

class TestSafeCost : public SafeCost {
public:
  TestSafeCost(const Costing& costing_options)
      : SafeCost(costing_options, CreatePedestrianCost(costing_options)) {};
  void TestH3();
};

void TestSafeCost::TestH3() {
}

TEST(SafeCost, testGraphToH3) {
  Costing c;
  c.set_type(Costing::safe);
  c.mutable_options()->set_flow_mask(kDefaultFlowMask);
  auto test = TestSafeCost(c);
  test.TestH3();
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#endif