#include "baldr/directededge.h"
#include "baldr/graphconstants.h"
#include "baldr/graphid.h"
#include "baldr/nodeinfo.h"
#include "proto/options.pb.h"
#include "sif/dynamiccost.h"
#include "sif/pedestriancost.h"
#include "proto_conversions.h"
#include <boost/container/flat_map.hpp>
#include <memory>
#include <pqxx/pqxx>
#include <unordered_map>
#include <h3/h3api.h>
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
// TODO - can we define these in dynamiccost.h and override here if they differ?
constexpr float kDefaultGatePenalty = 10.0f;           // Seconds
constexpr float kDefaultPrivateAccessPenalty = 600.0f; // Seconds
constexpr float kDefaultBssCost = 120.0f;              // Seconds
constexpr float kDefaultBssPenalty = 0.0f;             // Seconds
constexpr float kDefaultServicePenalty = 0.0f;         // Seconds

// Maximum route distances
constexpr uint32_t kMaxDistanceFoot = 100000;      // 100 km
constexpr uint32_t kMaxDistanceWheelchair = 10000; // 10 km

// Default speeds
constexpr float kDefaultSpeedFoot = 5.1f;       // 3.16 MPH
constexpr float kDefaultSpeedWheelchair = 4.0f; // 2.5  MPH  TODO

// Penalty to take steps
constexpr float kDefaultStepPenaltyFoot = 30.0f;        // 30 seconds
constexpr float kDefaultStepPenaltyWheelchair = 600.0f; // 10 minutes

// Penalty to take elevator
constexpr float kDefaultElevatorPenalty = 5.0f; // 5 seconds

// Maximum grade30
constexpr uint32_t kDefaultMaxGradeFoot = 90;
constexpr uint32_t kDefaultMaxGradeWheelchair = 12; // Conservative for now...

// Other defaults (not dependent on type)
constexpr uint8_t kDefaultMaxHikingDifficulty = 1; // T1 (kHiking)
constexpr float kModeFactor = 1.5f;                // Favor this mode?
constexpr float kDefaultWalkwayFactor = 1.0f;      // Neutral value for walkways
constexpr float kDefaultSideWalkFactor = 1.0f;     // Neutral value for sidewalks
constexpr float kDefaultAlleyFactor = 2.0f;        // Avoid alleys
constexpr float kDefaultDrivewayFactor = 5.0f;     // Avoid driveways
constexpr float kDefaultUseFerry = 1.0f;
constexpr float kDefaultUseLivingStreets = 0.6f; // Factor between 0 and 1

// Maximum distance at the beginning or end of a multimodal route
// that you are willing to travel for this mode.  In this case,
// it is the max walking distance.
constexpr uint32_t kTransitStartEndMaxDistance = 2415; // 1.5 miles

// Maximum transfer distance between stops that you are willing
// to travel for this mode.  In this case, it is the max walking
// distance you are willing to walk between transfers.
constexpr uint32_t kTransitTransferMaxDistance = 805; // 0.5 miles

// Avoid roundabouts
constexpr float kRoundaboutFactor = 2.0f;

// Minimum and maximum average pedestrian speed (to validate input).
constexpr float kMinPedestrianSpeed = 0.5f;
constexpr float kMaxPedestrianSpeed = 25.0f;

// Crossing penalties. TODO - may want to lower stop impact when
// 2 cycleways or walkways cross
constexpr uint32_t kCrossingCosts[] = {0, 0, 1, 1, 2, 3, 5, 15};

constexpr float kMinFactor = 0.1f;
constexpr float kMaxFactor = 100000.0f;

const std::string kDefaultPedestrianType = "foot";

// User propensity to use "hilly" roads. Ranges from a value of 0 (avoid
// hills) to 1 (take hills when they offer a more direct, less time, path).
constexpr float kDefaultUseHills = 0.5f;

constexpr float kDefaultUseLit = 0.f;

BaseCostingOptionsConfig GetBaseCostOptsConfig() {
  BaseCostingOptionsConfig cfg{};
  // override defaults
  cfg.gate_penalty_.def = kDefaultGatePenalty;
  cfg.private_access_penalty_.def = kDefaultPrivateAccessPenalty;
  cfg.disable_toll_booth_ = true;
  cfg.disable_rail_ferry_ = true;
  cfg.service_penalty_.def = kDefaultServicePenalty;
  cfg.use_ferry_.def = kDefaultUseFerry;
  cfg.use_living_streets_.def = kDefaultUseLivingStreets;
  cfg.use_lit_.def = kDefaultUseLit;
  return cfg;
}

} // namespace
const BaseCostingOptionsConfig kBaseCostOptsConfig = GetBaseCostOptsConfig();

struct CrimeData {
  float safety_factor;

  CrimeData(float safety_factor = 0.0f) : safety_factor(safety_factor) {
  }
};

class SafeCost : public DynamicCost {
public:
  SafeCost(const Costing& costing, std::shared_ptr<DynamicCost> base_costing);

  explicit SafeCost(const Costing& costing);
  virtual ~SafeCost() {

  };

  Cost
  TransitionCost(const baldr::DirectedEdge*, const baldr::NodeInfo*, const EdgeLabel&) const override;

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

  float AStarCostFactor() const override;

protected:
  static uint64_t GetH3(const baldr::DirectedEdge* edge, const graph_tile_ptr tile);
  std::shared_ptr<DynamicCost> base_costing_;
  static boost::container::flat_map<uint64_t, CrimeData> crime_data_;

private:
  float InterpolateSafety(float crime_rate) const;
  float max_safety_multiplier_ = 2.5f;

  static void LoadCrimeData(){
    if (crime_data_.empty()){

    }
  }

  Cost MutateCost(const baldr::DirectedEdge* edge, const graph_tile_ptr tile, const Cost base_cost) const;
};

boost::container::flat_map<uint64_t, CrimeData> SafeCost::crime_data_;

SafeCost::SafeCost(const Costing& costing, std::shared_ptr<DynamicCost> base_costing)
    : DynamicCost(costing, base_costing->travel_mode(), base_costing->access_mode()),
      base_costing_(base_costing) {
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

  return MutateCost(edge, tile, base_cost);
}


Cost SafeCost::EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::TransitDeparture* departure,
                        const uint32_t curr_time) const {
  throw std::runtime_error("SafeCost::EdgeCost does not support transit edges");
}

Cost SafeCost::TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const {
  return base_costing_->TransitionCost(edge, node, pred);
}

Cost SafeCost::MutateCost(const baldr::DirectedEdge* edge, const graph_tile_ptr tile, const Cost base_cost) const {
  float safety_factor = GetSafetyFactor(edge, tile);

  float adjusted_cost = base_cost.cost * (1.0f + (safety_factor - 1.0f));

  return {adjusted_cost, base_cost.secs};
}

float SafeCost::GetSafetyFactor(const baldr::DirectedEdge* edge, const graph_tile_ptr tile) const {
  uint64_t tile_key = GetH3(edge, tile);

  auto it = crime_data_.find(tile_key);
  if (it != crime_data_.end()) {
    return std::min(it->second.safety_factor, max_safety_multiplier_);
  }

  return 1.0f;
}

uint64_t SafeCost::GetH3(const baldr::DirectedEdge* edge, const graph_tile_ptr tile) {
  auto edgeInfo = tile->edgeinfo(edge);
  auto shape = edgeInfo.shape();

  double lat = 0;
  double lon = 0;
  for(auto coord : shape){
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
  if (crime_rate <= 5.0f)
    return 1.0f; // Very safe areas
  if (crime_rate <= 15.0f)
    return 1.2f; // Moderately safe
  if (crime_rate <= 30.0f)
    return 1.5f; // Some concern
  if (crime_rate <= 50.0f)
    return 2.0f;                 // Higher risk
  return max_safety_multiplier_; // Very dangerous
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

float SafeCost::AStarCostFactor() const {
  return 0;
}

void ParseSafeCostOptions(const rapidjson::Document& doc,
                                const std::string& costing_options_key,
                                Costing* c) {
  c->set_type(Costing::safe);
  c->set_name(Costing_Enum_Name(c->type()));
//  auto* co = c->mutable_options();
//
//  rapidjson::Value dummy;
//  const auto& json = rapidjson::get_child(doc, costing_options_key.c_str(), dummy);

  // TODO: custom safe costing opts? i.e. safe_coef?
}


cost_ptr_t CreateSafeCost(const Costing& costing_options) {
  return std::make_shared<SafeCost>(costing_options, CreatePedestrianCost(costing_options));
}
} // namespace sif
} // namespace valhalla

#ifdef INLINE_TEST
using namespace valhalla;
using namespace sif;
namespace {

class TestSafeCost : public SafeCost{
public:
  TestSafeCost(const Costing& costing_options) : SafeCost(costing_options, CreatePedestrianCost(costing_options)){};
  void TestH3();
};

void TestSafeCost::TestH3() {

}

TEST(SafeCost, testGraphToH3){
  Costing c;
  c.set_type(Costing::safe);
  c.mutable_options()->set_flow_mask(kDefaultFlowMask);
  auto test = TestSafeCost(c);
  test.TestH3();
}
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#endif