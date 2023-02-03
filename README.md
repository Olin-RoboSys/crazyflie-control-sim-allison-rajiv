 # 1D Crazyflie Altitude Control
 Rajiv Perera & Allison Li

# Part 1: Controller
To achieve the goal-criteria as closely as possible we remove the positive thrust limit located in the `utils.py` to 10x the force of gravity rather than 2.5x. We did not alter the minimum thrust, as this would have made the controller unrealistic (we are under the assumption that the propellers cannot provide negative thrust). 

Implementing the controller class was done  in the `Controller.py` file. 
```
    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """
        # e(t)
        err = setpoint.z_pos - state.z_pos
        # Derivative of e(t)
        derr_dt = setpoint.z_vel - state.z_vel
        # Integral of e(t)
        self.total_err += err
        
        z_dotdot = self.kp_z * err + self.kd_z * derr_dt + self.ki_z * self.total_err
        U = self.params.mass * (z_dotdot + self.params.g)

        return U
```

We used the equations of motion for control that were derived in class in conjunction with an error-term to use coefficients for k~p, k~i, k~d to tune the controller. We integrated the error over a period of time by adding the state-error to its previous integral value each timestep, and took the derivative of error by evaluating the change in z-velocity each timestep.  



## Task 1 & 2: PID Tuning

#Procedure 
We followed the procedure outlined in the Northwestern Robotics video "Empirical Gain Tuning" - we used steps in the order-of-magnitude to sanity check each term as we began experimenting with tuning. This procedure allowed us to sanity check the code as well as our commanded parameters - see the reflection section for more on this. 

### P vs PI vs PD vs PID 

P-only tuning was ineffective in this case because of limitations put on the thrust of the drone. Because the minimum thrust of the drone could never be in the negative-z direction, the thrust could never account for overshoot in a P-controller. The overshoot was at 21 meters - it took too long to slow down, because negative thrust after the drone passed the goal-point. Similarly, when falling to the goal point after overshoot it did not commit positive thrust until far too late. 

P-D tuning was more effective because the controller could take into account more criteria. The inclusion ofthe D-factor K_i allowed 

P-I tuning allowed for 

P-I-D tuning was the most effective model because 
## Task 3: Disturbance

## Reflection
### General realizations
There were several moments in which either the graphical output of the model or print-based debugging revealed aspects of the model which reflected true drone-control. 

After constructing the control class, initial tests were confusing - we were including damping terms, but the damping was not behaving as we had expected. The fluctuation in z position was becoming greater rather than smaller. Even more confusing, the damping term only actually seemed to damp when it was negative. After experimenting with different term combinations, we realized through print-based debugging that we were passing the k~d term to k~i, and passing 0 to the k~d term. In fact, we were never damping. Continuously adding the error together led to a compounding k~i term, explaining the behavior we were experiencing. 

Furthermore, the next graphical realization was an oversight in initial model constraints. Our drones kept falling through the floor, and oscillating in a downwards direction. This was because we had not accounted for the force of gravity in our controller, and were simply attempting to move the drone proportionally towards the goal without in addition resisting the pull of gravity. 

Lastly, the realization that minimum z-thrust was 0 rather than negative led to a diffferent approach to tuning the model - we realized that P-only controllers would not be effective, and stopped blaming the numbers that we were choosing. 

### Learning individually and together

This project was completed minus the disturbance-flag section in 2 2-hour meetings. 30 minutes was spent in understanding the code structure, approximately 45 minutes was used to create the controller class, and the remaining time was used in debugging, testing, and tuning values. All major code work was done during these meetings, but we consulted Grant when we ran into roadblocks asynchronously. It was very useful to compare notes to someone attempting to solve the same problem with similar baseline code. 

Both Allison and Rajiv are in ESA this semester, so are working with differential equations for control for the first time in a mathematics class at the same time as this class. In addition, neither of them have taken controls. This was a good introduction to basic control theory - the lectures were clear, and the application solidified what was learned. 


Rajiv was starting with a very baseline level of code skill and understanding on a basic architecture, methodology, and vocabulary at the beginning of this project, and spent much of the project helping debug the code as it was being written but also asking questions about the way that the code was structured. Exiting this project, he feels much more capable taking a system of files, understanding which is doing/storing what, and using vscode commands, github navigation, and the internet to find the solution to basic code issues. 

Allison was starting with a higher level of python knowledge than Rajiv, and was a very patient teacher. 

PID controllers are much more trial-and-error based than we initially expected - we were both surprised by the agression with which the example resources tuned their PID values, but this approach proved to be the most efficient. 

- What did you learn from this? What did you not know before this assignment?
- What was the most difficult aspect of the assignment?
- What was the easiest or most straightforward aspect of the assignment?
- How long did this assignment take? What took the most time (Setup? Figuring out the codebase? Coding in Python? Exploring the questions?)?
- What did you learn about PID controllers that we didn’t explicitly cover in class or in this assignment?
- What more would you like to learn about controls?

# Part 2: State Estimation

## Task 1 & 2: State Estimator

## Task 3: PID Controller with State Estimation

## Reflection
- What did you learn from this? What did you not know before this assignment (esp. Part 2)?
- What did you learn about kalman filters that we didn’t explicitly cover in class or in this assignment?
- What more would you like to learn about controls and/or state estimation?