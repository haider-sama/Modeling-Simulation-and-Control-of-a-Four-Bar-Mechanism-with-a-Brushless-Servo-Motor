# ⚙️ Four-Bar Mechanism with Brushless Servo Motor – MATLAB & Simulink

## 📖 Introduction
This project demonstrates the **dynamic modeling, simulation, and control** of a **four-bar mechanism** driven by a **brushless servo motor**, developed as part of the **MCT-241: Modelling & Simulation** course.  

The system was designed to analyze **nonlinear dynamics**, implement a **state-space model**, and evaluate mechanism behavior under different operating conditions. Both **linearized** and **nonlinear models** were developed, providing a comparative study of **accuracy vs. computational efficiency**.  

The main objective was to achieve **precise motion control** of the mechanism while accounting for realistic motor and linkage dynamics.  

---

## 🔬 Methodology
### 1. **System Modeling**
- Derived mathematical models for the **four-bar mechanism** and **brushless servo motor**.  
- Considered link lengths, moments of inertia, and motor constants.  
- Built **nonlinear state-space equations** to preserve system accuracy.  

### 2. **Simulation**
- Implemented both **linear** and **nonlinear models** in **MATLAB & Simulink**.  
- Simulated angular positions, velocities, motor torque, and current.  
- Compared the **linear model** (simpler, faster) with the **nonlinear model** (more accurate).  

### 3. **Control System Design**
- Designed and tested **control strategies** for tracking and stability.  
- Evaluated controller performance under nonlinear system effects.  

---

## 🏗️ Applications
- 🤖 **Robotics**  
- 🚗 **Automotive Systems**  
- 🏭 **Industrial Machinery & Automation**  

---

## 🗂️ Project Structure
<br />
├──📂 MatLab_Code                          # MATLAB implementation (linear & nonlinear models)  
<br />
├──📂 Presentation                         # Project presentation slides  
<br />
├──📄 CEP_MS_2022_MC_45.pdf                # Full project report  
<br />
├──📄 CEP_MS_2022_MC_45_Simulation_Results.pdf   # Simulation results  
<br />
├──📄 README.md                            # Project README  

---

📌 Key Insights  
<br />
- **Nonlinear modeling** captures realistic effects like torque oscillations and nonlinear inertia.  
- **Linear models** are faster to simulate but lose accuracy in complex or high-speed scenarios.  
- Demonstrates the trade-off between **computational simplicity** and **modeling fidelity**.  
- Provides a foundation for **advanced control strategies** in servo-driven mechanisms.  

---

## 📝 Conclusion
This project successfully demonstrated the **dynamic modeling, simulation, and control** of a **servo motor-driven four-bar mechanism**.  

- The **nonlinear model** provided high-fidelity representation of system dynamics.  
- The **linear model** worked as a simplified baseline but was less accurate in nonlinear conditions.  
- Results emphasized the importance of nonlinear modeling for **precision control** and **performance optimization**.  

---

## 📚 References
- Course: **MCT-241: Modelling & Simulation**  
- Department of Mechatronics & Control Engineering, UET Lahore  
