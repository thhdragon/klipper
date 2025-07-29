## Key Focus Areas

### 1. Mathematical Logic Extraction Over Implementation
- **DO**: Extract kinematic equations and derive optimal control algorithms
- **DON'T**: Document Klipper's specific file structure or API calls
- **Example**: "Given a CoreXY kinematics system, derive the transformation matrix and prove that the system maintains orthogonal motion. How would Rust's type system prevent invalid coordinate transformations at compile time?"# 3D Printer LLM Agent Dataset Generation Prompt

## Mission Statement
You are tasked with creating a comprehensive dataset optimized for training **Phi-4-mini-flash-reasoning** on advanced 3D printer control systems. Your role is to extract universal logic, reasoning patterns, and problem-solving methodologies from Klipper's source code that can be applied to any 3D printer software or custom printer OS, with emphasis on **Rust programming paradigms** and **mathematical reasoning chains**.

## Core Objective for Phi-4-mini-flash-reasoning
Extract the **logic and reasoning** behind Klipper's operations through **step-by-step mathematical and logical reasoning chains**. The model is trained on synthetic mathematical content from advanced reasoning models, so structure your dataset to match this reasoning-focused approach. Emphasize **Rust's ownership model, memory safety, and concurrent programming patterns** as they apply to real-time 3D printer control.

## Phi-4-Mini-Flash-Reasoning Optimization Strategy

### Mathematical Reasoning Focus
The model is trained exclusively on synthetic mathematical content from advanced reasoning models, so structure each dataset entry as:
- **Mathematical formulation** of the problem
- **Step-by-step logical reasoning** chains
- **Proof-like demonstrations** of correctness
- **Edge case analysis** through mathematical bounds

### Rust Programming Integration
Since Klipper uses Python for high-level logic but modern 3D printer firmware can benefit from Rust's memory safety and performance, frame reasoning problems to teach:
- **Ownership and borrowing** for safe hardware resource management
- **Concurrent programming** patterns for real-time control loops
- **Zero-cost abstractions** for efficient motor control algorithms
- **Type system guarantees** for preventing runtime errors in critical systems

### Reasoning Chain Structure
Follow patterns that mirror mathematical proof methodology:
1. **Problem Definition**: State the control system challenge mathematically
2. **Constraints & Assumptions**: Define physical and computational limits
3. **Logical Derivation**: Step-by-step reasoning toward solution
4. **Rust Implementation Logic**: How Rust's features enable safe implementation
5. **Verification**: Mathematical proof of correctness or safety bounds

### 1. Logic Extraction Over Implementation
- **DO**: Extract the reasoning behind motion planning algorithms
- **DON'T**: Document Klipper's specific file structure or API calls
- **Example**: Instead of "Klipper uses trapezoid.py for motion", extract "How should a printer OS calculate acceleration curves to minimize vibration while maximizing speed?"

### 2. Rust-Centric Universal Principles
- Extract concepts applicable to any real-time control system
- Focus on memory-safe concurrent programming patterns
- Emphasize zero-cost abstractions and compile-time guarantees
- Demonstrate how Rust's ownership model prevents common embedded systems bugs

### 3. Mathematical Reasoning-Style Query Generation
Structure findings as mathematical proofs and logical derivations that:
- Present formal problem statements with clear mathematical notation
- Build reasoning chains step-by-step with logical justification
- Include Rust code patterns that enforce mathematical constraints
- Demonstrate verification through type system guarantees

## Dataset Categories to Generate

## Dataset Categories with Rust Integration

### Motion Control & Kinematics with Mathematical Proofs
- **Mathematical Formulation**: Derive motion equations and prove stability bounds
- **Rust Safety Logic**: How ownership prevents simultaneous motor access conflicts
- **Concurrency Patterns**: Lock-free algorithms for real-time motion planning
- **Type Safety**: Using Rust's type system to enforce physical constraints (positive velocities, bounded accelerations)

### Thermal Management with Control Theory
- **PID Mathematical Analysis**: Derive optimal PID parameters through mathematical optimization
- **Rust Memory Safety**: Safe handling of temperature sensor data without buffer overflows
- **Concurrent Temperature Monitoring**: Async/await patterns for non-blocking thermal management
- **Compile-time Verification**: Using const generics to enforce temperature bounds

### Error Handling & Recovery with Formal Verification
- **State Machine Proofs**: Mathematical verification of state transition safety
- **Rust Error Propagation**: Using Result<T,E> for safe error handling without panics
- **Resource Management**: RAII patterns for hardware resource cleanup
- **Deadlock Prevention**: Mathematical proof of deadlock-freedom in concurrent systems

### Real-Time Control Systems with Performance Guarantees
- **Timing Analysis**: Mathematical bounds on worst-case execution time
- **Zero-Cost Rust Abstractions**: Compile-time optimizations that maintain real-time guarantees
- **Memory Pool Management**: Lock-free allocators for deterministic memory access
- **Interrupt Safety**: Proving interrupt handler correctness through Rust's type system

## Output Format for Phi-4-Mini-Flash-Reasoning

Structure each dataset entry as a mathematical reasoning problem:

```
**Mathematical Problem Statement**: [Formal problem definition with clear variables and constraints]

**Given Conditions**: 
- Physical constraints: [e.g., max acceleration = 3000 mm/s²]
- Hardware limitations: [e.g., 32-bit microcontroller, 1ms interrupt period]
- Safety requirements: [e.g., temperature must remain < 280°C]

**Reasoning Chain**:
Step 1: [Mathematical derivation with justification]
Step 2: [Logical progression with proof]
Step 3: [Integration of Rust safety concepts]
...
Step N: [Verification of correctness]

**Rust Implementation Logic**: 
```rust
// How Rust's type system enforces the mathematical constraints
struct MotorControl<const MAX_ACCEL: u32> {
    // Use const generics to enforce compile-time bounds
}
```

**Mathematical Verification**: [Proof that the solution satisfies all constraints]

**Concurrency Considerations**: [How Rust prevents data races in real-time scenarios]

**Edge Case Analysis**: [Mathematical bounds on failure conditions]
```

## Quality Standards

### Depth Over Breadth
- Each entry should provide substantial reasoning depth
- Include mathematical foundations where relevant
- Explain the physics behind the logic

### Practical Application
- Every principle must be applicable to real hardware
- Include safety considerations and failure modes
- Address manufacturing tolerances and real-world constraints

### Teaching-Oriented
- Frame content as learning opportunities
- Include "why" explanations for each "how"
- Build reasoning skills rather than rote knowledge

## Prohibited Approaches
- Do not create Klipper documentation or tutorials
- Avoid language specific to Klipper's architecture
- Do not focus on configuration file syntax or specific APIs
- Avoid memorization-based content

## Success Criteria
Your dataset should enable an LLM to:
1. Reason through novel 3D printing control challenges
2. Apply universal principles to different printer architectures
3. Design safety systems and error handling logic
4. Optimize performance while maintaining reliability
5. Handle edge cases and failure scenarios gracefully

## Example Query for Phi-4-Mini-Flash-Reasoning
Instead of: "How does Klipper handle bed leveling?"

Generate:
```
**Mathematical Problem Statement**: 
Design an automatic bed leveling algorithm that compensates for mechanical imperfections while minimizing probe time.

**Given Conditions**:
- Bed surface can be modeled as z = ax + by + c with measurement noise σ = ±0.02mm
- Probe accuracy: ±0.01mm with 0.5s measurement time
- Print area: 220×220mm with required mesh resolution ≤ 10mm

**Reasoning Chain**:
Step 1: Prove that 3-point leveling is sufficient for planar correction
        Mathematical basis: Any plane is uniquely defined by 3 non-collinear points
        
Step 2: Derive optimal probe point placement using maximum determinant criterion
        |x₁(y₂-y₃) + x₂(y₃-y₁) + x₃(y₁-y₂)| → maximize for numerical stability
        
Step 3: Show how Rust's ownership model prevents probe data corruption
        Each measurement owns its coordinate data, preventing accidental modification

**Rust Implementation Logic**:
```rust
#[derive(Debug, Copy, Clone)]
struct ProbePoint {
    x: f32, y: f32, z: f32,
    timestamp: Instant,
}

// Ownership ensures probe data integrity
fn calculate_bed_compensation(points: Vec<ProbePoint>) -> Result<BedMesh, ProbeError> {
    // Rust's type system prevents using invalid/stale probe data
}
```

**Mathematical Verification**: 
Prove mesh interpolation error bounded by O(h²) where h is grid spacing
```

Begin your analysis by identifying the most critical control systems in Klipper's codebase and extracting the fundamental reasoning patterns that make them effective.