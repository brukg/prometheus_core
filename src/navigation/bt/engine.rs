//! Core behavior tree engine: tick/halt semantics, node types, blackboard.

use std::any::Any;
use std::collections::HashMap;

/// The result of ticking a BT node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BtStatus {
    Success,
    Failure,
    Running,
}

/// A type-erased blackboard for sharing data between BT nodes.
pub struct Blackboard {
    data: HashMap<String, Box<dyn Any + Send + Sync>>,
}

impl Blackboard {
    pub fn new() -> Self {
        Self {
            data: HashMap::new(),
        }
    }

    pub fn set<T: Any + Send + Sync>(&mut self, key: &str, value: T) {
        self.data.insert(key.to_string(), Box::new(value));
    }

    pub fn get<T: Any + Send + Sync>(&self, key: &str) -> Option<&T> {
        self.data.get(key).and_then(|v| v.downcast_ref::<T>())
    }

    pub fn get_mut<T: Any + Send + Sync>(&mut self, key: &str) -> Option<&mut T> {
        self.data.get_mut(key).and_then(|v| v.downcast_mut::<T>())
    }

    pub fn contains_key(&self, key: &str) -> bool {
        self.data.contains_key(key)
    }

    pub fn remove(&mut self, key: &str) {
        self.data.remove(key);
    }
}

impl Default for Blackboard {
    fn default() -> Self {
        Self::new()
    }
}

/// Trait for action nodes (leaf nodes that do work).
pub trait BtAction: Send + Sync {
    fn tick(&mut self, blackboard: &mut Blackboard) -> BtStatus;
    fn halt(&mut self);
    fn name(&self) -> &str;
}

/// Trait for condition nodes (leaf nodes that check state).
pub trait BtCondition: Send + Sync {
    fn check(&self, blackboard: &Blackboard) -> bool;
    fn name(&self) -> &str;
}

/// Trait for decorator nodes (wrap a single child).
pub trait BtDecorator: Send + Sync {
    fn tick(&mut self, child_status: BtStatus, blackboard: &mut Blackboard) -> BtStatus;
    fn name(&self) -> &str;
}

/// A behavior tree node.
pub enum BtNode {
    /// Leaf action node.
    Action(Box<dyn BtAction>),
    /// Leaf condition node.
    Condition(Box<dyn BtCondition>),
    /// Sequence: ticks children left-to-right. Returns Failure on first Failure,
    /// Running on first Running, Success only if all children succeed.
    Sequence(Vec<BtNode>),
    /// Fallback (Selector): ticks children left-to-right. Returns Success on first
    /// Success, Running on first Running, Failure only if all children fail.
    Fallback(Vec<BtNode>),
    /// Decorator: wraps a single child with custom logic.
    Decorator(Box<dyn BtDecorator>, Box<BtNode>),
    /// PipelineSequence: like Sequence but re-ticks already-completed children.
    PipelineSequence(Vec<BtNode>),
    /// RecoveryNode: tries main, on failure tries recovery, retries up to N times.
    RecoveryNode {
        main: Box<BtNode>,
        recovery: Box<BtNode>,
        max_retries: u32,
        current_retries: u32,
    },
    /// RoundRobin: cycles through children, starting from where it left off.
    RoundRobin {
        children: Vec<BtNode>,
        current_child: usize,
    },
}

impl BtNode {
    /// Tick the behavior tree node.
    pub fn tick(&mut self, blackboard: &mut Blackboard) -> BtStatus {
        match self {
            BtNode::Action(action) => action.tick(blackboard),

            BtNode::Condition(condition) => {
                if condition.check(blackboard) {
                    BtStatus::Success
                } else {
                    BtStatus::Failure
                }
            }

            BtNode::Sequence(children) => {
                for child in children.iter_mut() {
                    match child.tick(blackboard) {
                        BtStatus::Success => continue,
                        BtStatus::Running => return BtStatus::Running,
                        BtStatus::Failure => return BtStatus::Failure,
                    }
                }
                BtStatus::Success
            }

            BtNode::Fallback(children) => {
                for child in children.iter_mut() {
                    match child.tick(blackboard) {
                        BtStatus::Failure => continue,
                        BtStatus::Running => return BtStatus::Running,
                        BtStatus::Success => return BtStatus::Success,
                    }
                }
                BtStatus::Failure
            }

            BtNode::Decorator(decorator, child) => {
                let child_status = child.tick(blackboard);
                decorator.tick(child_status, blackboard)
            }

            BtNode::PipelineSequence(children) => {
                // Re-tick all children, including completed ones
                for child in children.iter_mut() {
                    match child.tick(blackboard) {
                        BtStatus::Failure => return BtStatus::Failure,
                        BtStatus::Running => return BtStatus::Running,
                        BtStatus::Success => continue,
                    }
                }
                BtStatus::Success
            }

            BtNode::RecoveryNode {
                main,
                recovery,
                max_retries,
                current_retries,
            } => {
                // Try main
                match main.tick(blackboard) {
                    BtStatus::Success => {
                        *current_retries = 0;
                        BtStatus::Success
                    }
                    BtStatus::Running => BtStatus::Running,
                    BtStatus::Failure => {
                        if *current_retries >= *max_retries {
                            *current_retries = 0;
                            return BtStatus::Failure;
                        }

                        // Try recovery
                        match recovery.tick(blackboard) {
                            BtStatus::Success => {
                                *current_retries += 1;
                                // Retry main on next tick
                                BtStatus::Running
                            }
                            BtStatus::Running => BtStatus::Running,
                            BtStatus::Failure => {
                                *current_retries = 0;
                                BtStatus::Failure
                            }
                        }
                    }
                }
            }

            BtNode::RoundRobin {
                children,
                current_child,
            } => {
                if children.is_empty() {
                    return BtStatus::Failure;
                }

                let start = *current_child;
                loop {
                    match children[*current_child].tick(blackboard) {
                        BtStatus::Success => {
                            *current_child = (*current_child + 1) % children.len();
                            return BtStatus::Success;
                        }
                        BtStatus::Running => return BtStatus::Running,
                        BtStatus::Failure => {
                            *current_child = (*current_child + 1) % children.len();
                            if *current_child == start {
                                // All children failed
                                return BtStatus::Failure;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Halt the behavior tree node and all descendants.
    pub fn halt(&mut self) {
        match self {
            BtNode::Action(action) => action.halt(),
            BtNode::Condition(_) => {}
            BtNode::Sequence(children) | BtNode::Fallback(children) | BtNode::PipelineSequence(children) => {
                for child in children.iter_mut() {
                    child.halt();
                }
            }
            BtNode::Decorator(_, child) => child.halt(),
            BtNode::RecoveryNode { main, recovery, current_retries, .. } => {
                main.halt();
                recovery.halt();
                *current_retries = 0;
            }
            BtNode::RoundRobin { children, current_child } => {
                for child in children.iter_mut() {
                    child.halt();
                }
                *current_child = 0;
            }
        }
    }
}

// --- Built-in decorators ---

/// RateController: throttles child tick rate.
pub struct RateController {
    name: String,
    /// Minimum interval between ticking the child.
    min_interval: std::time::Duration,
    last_tick: Option<std::time::Instant>,
    last_status: BtStatus,
}

impl RateController {
    pub fn new(name: &str, rate_hz: f64) -> Self {
        Self {
            name: name.to_string(),
            min_interval: std::time::Duration::from_secs_f64(1.0 / rate_hz),
            last_tick: None,
            last_status: BtStatus::Running,
        }
    }
}

impl BtDecorator for RateController {
    fn tick(&mut self, child_status: BtStatus, _blackboard: &mut Blackboard) -> BtStatus {
        let now = std::time::Instant::now();
        let should_tick = match self.last_tick {
            Some(last) => now.duration_since(last) >= self.min_interval,
            None => true,
        };

        if should_tick {
            self.last_tick = Some(now);
            self.last_status = child_status;
        }

        self.last_status
    }

    fn name(&self) -> &str {
        &self.name
    }
}

// --- Helper constructors ---

/// Create a RecoveryNode.
pub fn recovery_node(main: BtNode, recovery: BtNode, max_retries: u32) -> BtNode {
    BtNode::RecoveryNode {
        main: Box::new(main),
        recovery: Box::new(recovery),
        max_retries,
        current_retries: 0,
    }
}

/// Create a RoundRobin node.
pub fn round_robin(children: Vec<BtNode>) -> BtNode {
    BtNode::RoundRobin {
        children,
        current_child: 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test action that returns a fixed status
    struct FixedAction {
        status: BtStatus,
        name: String,
        ticked: bool,
    }

    impl FixedAction {
        fn new(name: &str, status: BtStatus) -> Self {
            Self {
                status,
                name: name.to_string(),
                ticked: false,
            }
        }
    }

    impl BtAction for FixedAction {
        fn tick(&mut self, _blackboard: &mut Blackboard) -> BtStatus {
            self.ticked = true;
            self.status
        }
        fn halt(&mut self) {
            self.ticked = false;
        }
        fn name(&self) -> &str {
            &self.name
        }
    }

    // Test action that succeeds after N ticks
    struct CountdownAction {
        name: String,
        remaining: u32,
        initial: u32,
    }

    impl CountdownAction {
        fn new(name: &str, ticks: u32) -> Self {
            Self {
                name: name.to_string(),
                remaining: ticks,
                initial: ticks,
            }
        }
    }

    impl BtAction for CountdownAction {
        fn tick(&mut self, _blackboard: &mut Blackboard) -> BtStatus {
            if self.remaining == 0 {
                return BtStatus::Success;
            }
            self.remaining -= 1;
            BtStatus::Running
        }
        fn halt(&mut self) {
            self.remaining = self.initial;
        }
        fn name(&self) -> &str {
            &self.name
        }
    }

    // Test condition
    struct KeyExistsCondition {
        key: String,
    }

    impl BtCondition for KeyExistsCondition {
        fn check(&self, blackboard: &Blackboard) -> bool {
            blackboard.contains_key(&self.key)
        }
        fn name(&self) -> &str {
            "KeyExists"
        }
    }

    #[test]
    fn test_action_success() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Action(Box::new(FixedAction::new("ok", BtStatus::Success)));
        assert_eq!(node.tick(&mut bb), BtStatus::Success);
    }

    #[test]
    fn test_action_failure() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Action(Box::new(FixedAction::new("fail", BtStatus::Failure)));
        assert_eq!(node.tick(&mut bb), BtStatus::Failure);
    }

    #[test]
    fn test_condition_true() {
        let mut bb = Blackboard::new();
        bb.set("key", 42);
        let mut node = BtNode::Condition(Box::new(KeyExistsCondition {
            key: "key".to_string(),
        }));
        assert_eq!(node.tick(&mut bb), BtStatus::Success);
    }

    #[test]
    fn test_condition_false() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Condition(Box::new(KeyExistsCondition {
            key: "missing".to_string(),
        }));
        assert_eq!(node.tick(&mut bb), BtStatus::Failure);
    }

    #[test]
    fn test_sequence_all_success() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Sequence(vec![
            BtNode::Action(Box::new(FixedAction::new("a", BtStatus::Success))),
            BtNode::Action(Box::new(FixedAction::new("b", BtStatus::Success))),
        ]);
        assert_eq!(node.tick(&mut bb), BtStatus::Success);
    }

    #[test]
    fn test_sequence_first_failure() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Sequence(vec![
            BtNode::Action(Box::new(FixedAction::new("a", BtStatus::Failure))),
            BtNode::Action(Box::new(FixedAction::new("b", BtStatus::Success))),
        ]);
        assert_eq!(node.tick(&mut bb), BtStatus::Failure);
    }

    #[test]
    fn test_sequence_running() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Sequence(vec![
            BtNode::Action(Box::new(FixedAction::new("a", BtStatus::Success))),
            BtNode::Action(Box::new(FixedAction::new("b", BtStatus::Running))),
        ]);
        assert_eq!(node.tick(&mut bb), BtStatus::Running);
    }

    #[test]
    fn test_fallback_first_success() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Fallback(vec![
            BtNode::Action(Box::new(FixedAction::new("a", BtStatus::Success))),
            BtNode::Action(Box::new(FixedAction::new("b", BtStatus::Failure))),
        ]);
        assert_eq!(node.tick(&mut bb), BtStatus::Success);
    }

    #[test]
    fn test_fallback_all_failure() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Fallback(vec![
            BtNode::Action(Box::new(FixedAction::new("a", BtStatus::Failure))),
            BtNode::Action(Box::new(FixedAction::new("b", BtStatus::Failure))),
        ]);
        assert_eq!(node.tick(&mut bb), BtStatus::Failure);
    }

    #[test]
    fn test_recovery_node_main_succeeds() {
        let mut bb = Blackboard::new();
        let mut node = recovery_node(
            BtNode::Action(Box::new(FixedAction::new("main", BtStatus::Success))),
            BtNode::Action(Box::new(FixedAction::new("recover", BtStatus::Success))),
            3,
        );
        assert_eq!(node.tick(&mut bb), BtStatus::Success);
    }

    #[test]
    fn test_recovery_node_triggers_recovery() {
        let mut bb = Blackboard::new();
        let mut node = recovery_node(
            BtNode::Action(Box::new(FixedAction::new("main", BtStatus::Failure))),
            BtNode::Action(Box::new(FixedAction::new("recover", BtStatus::Success))),
            3,
        );
        // First tick: main fails, recovery succeeds, returns Running (will retry)
        assert_eq!(node.tick(&mut bb), BtStatus::Running);
    }

    #[test]
    fn test_recovery_node_exhausts_retries() {
        let mut bb = Blackboard::new();
        let mut node = recovery_node(
            BtNode::Action(Box::new(FixedAction::new("main", BtStatus::Failure))),
            BtNode::Action(Box::new(FixedAction::new("recover", BtStatus::Success))),
            2,
        );
        // Tick until retries exhausted
        assert_eq!(node.tick(&mut bb), BtStatus::Running); // retry 1
        assert_eq!(node.tick(&mut bb), BtStatus::Running); // retry 2
        assert_eq!(node.tick(&mut bb), BtStatus::Failure); // exhausted
    }

    #[test]
    fn test_round_robin() {
        let mut bb = Blackboard::new();
        let mut node = round_robin(vec![
            BtNode::Action(Box::new(FixedAction::new("a", BtStatus::Success))),
            BtNode::Action(Box::new(FixedAction::new("b", BtStatus::Success))),
        ]);
        // First tick: runs child 0, succeeds, advances to 1
        assert_eq!(node.tick(&mut bb), BtStatus::Success);
        // Second tick: runs child 1, succeeds, advances to 0
        assert_eq!(node.tick(&mut bb), BtStatus::Success);
    }

    #[test]
    fn test_blackboard_operations() {
        let mut bb = Blackboard::new();
        bb.set("count", 42_i32);
        bb.set("name", "test".to_string());

        assert_eq!(*bb.get::<i32>("count").unwrap(), 42);
        assert_eq!(bb.get::<String>("name").unwrap(), "test");
        assert!(bb.get::<i32>("missing").is_none());
        assert!(bb.contains_key("count"));

        *bb.get_mut::<i32>("count").unwrap() = 100;
        assert_eq!(*bb.get::<i32>("count").unwrap(), 100);

        bb.remove("count");
        assert!(!bb.contains_key("count"));
    }

    #[test]
    fn test_countdown_action() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Action(Box::new(CountdownAction::new("timer", 2)));
        assert_eq!(node.tick(&mut bb), BtStatus::Running);
        assert_eq!(node.tick(&mut bb), BtStatus::Running);
        assert_eq!(node.tick(&mut bb), BtStatus::Success);
    }

    #[test]
    fn test_halt_resets_state() {
        let mut bb = Blackboard::new();
        let mut node = BtNode::Action(Box::new(CountdownAction::new("timer", 2)));
        node.tick(&mut bb); // Running
        node.tick(&mut bb); // Running
        node.halt();        // Reset
        assert_eq!(node.tick(&mut bb), BtStatus::Running); // Back to start
    }
}
