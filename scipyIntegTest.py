import numpy as np
from scipy.integrate import cumulative_trapezoid
import matplotlib.pyplot as plt

# Define the function to integrate
def f(x):
    return x**2

# Define the x values
x = np.linspace(0, 10, 1000)

# Compute y = f(x)
y = f(x)

# Compute the cumulative integral using trapezoidal rule
cumulative_integral = cumulative_trapezoid(y, x, initial=0)

# Compute the analytical solution: integral of x^2 = (1/3)x^3
analytical_solution = (1/3) * x**3

# Plot the numerical vs analytical solutions
plt.figure(figsize=(10, 5))
plt.plot(x, cumulative_integral, label='Cumulative Trapezoid', linewidth=2)
plt.plot(x, analytical_solution, '--', label='Analytical (1/3 x^3)', linewidth=2)
plt.legend()
plt.title("Cumulative Trapezoidal Integration vs Analytical Solution")
plt.xlabel("x")
plt.ylabel("Integral of x^2")
plt.grid(True)
plt.show()

# Print final values for quick verification
print(f"Final value (numerical): {cumulative_integral[-1]}")
print(f"Final value (analytical): {analytical_solution[-1]}")
print(f"Absolute error: {abs(cumulative_integral[-1] - analytical_solution[-1])}")
