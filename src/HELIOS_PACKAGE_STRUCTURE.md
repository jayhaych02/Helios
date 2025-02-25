# HELIOS_CORE

helios_core/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── core.hpp                      # Core definitions & constants
│   ├── robot_base.hpp                # Base robot class
│   ├── robot_types/                  # Robot type definitions
│   │   ├── robot_firefighter.hpp     # Firefighter robot implementation
│   │   ├── robot_scout.hpp           # Scout robot implementation
│   │   ├── robot_medical.hpp         # Medical robot implementation
│   │   └── robot_heavy_duty.hpp      # Heavy duty robot implementation
│   ├── state_estimation/             # Filtering & state estimation headers
│   │   ├── kalman_filter.hpp         # Basic Kalman Filter
│   │   ├── extended_kf.hpp           # Extended Kalman Filter
│   │   ├── unscented_kf.hpp          # Unscented Kalman Filter
│   │   └── particle_filter.hpp       # Particle Filter implementation
│   ├── coordination/                 # Multi-robot coordination headers
│   │   ├── market_based.hpp          # Market-based task allocation
│   │   ├── consensus.hpp             # Consensus algorithms
│   │   └── flocking.hpp              # Reynolds' flocking model
│   ├── disaster/                     # Disaster simulation headers
│   │   ├── fire_model.hpp            # Fire propagation algorithms
│   │   ├── flood_model.hpp           # Flood dynamics
│   │   └── building_collapse.hpp     # Structural failure simulation
│   ├── performance/                  # Evaluation and metrics headers
│   │   ├── metrics.hpp               # Performance metrics calculations
│   │   └── evaluation.hpp            # Evaluation framework
│   └── common/                       # Shared utilities headers
│       ├── math_utils.hpp            # Mathematical helpers
│       ├── transforms.hpp            # Coordinate transformations
│       └── params.hpp                # Parameter handling
└── src/
    ├── robot_implementations/        # Robot implementations
    │   ├── robot_firefighter.cpp     # Firefighter robot implementation
    │   ├── robot_scout.cpp           # Scout robot implementation
    │   ├── robot_medical.cpp         # Medical robot implementation
    │   └── robot_heavy_duty.cpp      # Heavy duty robot implementation
    ├── state_estimation/             # State estimation implementations
    │   ├── kalman_filter.cpp         # Kalman Filter implementation
    │   ├── extended_kf.cpp           # Extended Kalman Filter implementation
    │   ├── unscented_kf.cpp          # Unscented Kalman Filter implementation
    │   └── particle_filter.cpp       # Particle Filter implementation
    ├── coordination/                 # Coordination implementations
    │   ├── market_based.cpp          # Market-based allocation implementation
    │   ├── consensus.cpp             # Consensus algorithms implementation
    │   └── flocking.cpp              # Flocking behavior implementation
    ├── disaster/                     # Disaster model implementations
    │   ├── fire_model.cpp            # Fire simulation implementation
    │   ├── flood_model.cpp           # Flood simulation implementation
    │   └── building_collapse.cpp     # Building collapse implementation
    ├── performance/                  # Performance analysis implementations
    │   ├── metrics.cpp               # Performance metrics implementation
    │   └── evaluation.cpp            # Evaluation framework implementation
    └── common/                       # Common utilities
        ├── math_utils.cpp            # Math utilities implementation
        ├── transforms.cpp            # Transform utilities implementation
        └── params.cpp                # Parameter utilities implementation



# HELIOS NAVIGATION


helios_navigation/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── controllers/                  # Controller headers
│   │   ├── regulated_pp.hpp          # Regulated Pure Pursuit
│   │   ├── dwb_controller.hpp        # DWB implementation
│   │   └── mppi_controller.hpp       # MPPI controller
│   ├── planners/                     # Planner headers
│   │   ├── mstar_planner.hpp         # M* implementation
│   │   ├── rrt_planner.hpp           # RRT/RRT* implementation
│   │   └── navfn_adapter.hpp         # NavFn interface
│   ├── behaviors/                    # Recovery behavior headers
│   │   ├── spin_behavior.hpp         # Spin in place
│   │   └── backup_behavior.hpp       # Backup recovery
│   └── costmaps/                     # Costmap headers
│       ├── disaster_layer.hpp        # Disaster-specific costmap layer
│       └── multi_robot_layer.hpp     # Layer for tracking other robots
├── src/
│   ├── controllers/                  # Controller implementations
│   │   ├── regulated_pp.cpp          # Regulated Pure Pursuit implementation
│   │   ├── dwb_controller.cpp        # DWB controller implementation
│   │   └── mppi_controller.cpp       # MPPI controller implementation
│   ├── planners/                     # Planner implementations
│   │   ├── mstar_planner.cpp         # M* planner implementation
│   │   ├── rrt_planner.cpp           # RRT/RRT* planner implementation
│   │   └── navfn_adapter.cpp         # NavFn adapter implementation
│   ├── behaviors/                    # Recovery behavior implementations
│   │   ├── spin_behavior.cpp         # Spin behavior implementation
│   │   └── backup_behavior.cpp       # Backup behavior implementation
│   └── costmaps/                     # Costmap implementations
│       ├── disaster_layer.cpp        # Disaster layer implementation
│       └── multi_robot_layer.cpp     # Multi-robot layer implementation
├── plugins/
│   ├── controller_plugins.xml        # Controller plugin declarations
│   ├── planner_plugins.xml           # Planner plugin declarations
│   ├── behavior_plugins.xml          # Behavior plugin declarations
│   └── costmap_plugins.xml           # Costmap plugin declarations
└── config/
    ├── controller_config.yaml        # Controller configurations
    ├── planner_config.yaml           # Planner configurations
    ├── behavior_config.yaml          # Behavior configurations
    ├── costmap_config.yaml           # Costmap configurations
    └── navigation_config.yaml        # Overall navigation config