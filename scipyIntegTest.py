import numpy as np
from scipy.integrate import cumulative_trapezoid
import matplotlib.pyplot as plt

def f(x):
    return x**2

x = np.linspace(0, 10, 1000)

y = f(x)

cumulative_integral = cumulative_trapezoid(y, x, initial=0)

analytical_solution = (1/3) * x**3

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
