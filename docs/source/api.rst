API Reference
============

Core Components
-------------

PrometheusCore
~~~~~~~~~~~~~

The main class that manages all components of the robot.

.. code-block:: rust

   let mut core = PrometheusCore::new();
   core.register(NavigationStack::new());
   core.register(ControlStack::new());
   core.register(PerceptionStack::new());
   core.init()?;

Navigation
---------

NavigationStack
~~~~~~~~~~~~~

Handles path planning and following.

.. code-block:: rust

   let mut nav_stack = NavigationStack::with_path_follower(PurePursuitFollower::new());
   
   // Configure the path follower
   let mut params = HashMap::new();
   params.insert("lookahead_distance".to_string(), 0.8);
   nav_stack.configure_path_follower(&params)?;

PathFollower
~~~~~~~~~~

Trait for path following algorithms.

.. code-block:: rust

   pub trait PathFollower: Debug + Send + Sync {
       fn new() -> Self where Self: Sized;
       fn compute_velocity(&self, current_pose: (f64, f64, f64), path: &[(f64, f64)]) -> (f64, f64);
       fn name(&self) -> &str;
       fn configure(&mut self, params: &HashMap<String, f64>) -> Result<(), String>;
   }

Control
------

ControlStack
~~~~~~~~~~

Manages low-level control of the robot.

.. code-block:: rust

   let control_stack = ControlStack::new();

Perception
---------

PerceptionStack
~~~~~~~~~~~~

Handles sensor data processing and localization.

.. code-block:: rust

   let perception_stack = PerceptionStack::new();

Lifecycle Management
------------------

LifecycleNode
~~~~~~~~~~~

Trait for components that follow a lifecycle pattern.

.. code-block:: rust

   pub trait LifecycleNode: Send + Sync {
       fn on_configure(&mut self) -> Result<(), String>;
       fn on_activate(&mut self) -> Result<(), String>;
       fn on_deactivate(&mut self) -> Result<(), String>;
       fn on_cleanup(&mut self) -> Result<(), String>;
       fn get_name(&self) -> &str;
   } 