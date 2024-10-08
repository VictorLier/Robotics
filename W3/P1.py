import sympy as sp

# Question 2
# Define symbols
theta = list(sp.symbols('theta_1:7'))
d = list(sp.symbols('d_1:7'))
a = list(sp.symbols('a_1:7'))
alpha = list(sp.symbols('alpha_1:7'))
A = list (sp.symbols('A_1:7'))
T = sp.symbols('T')

for i in range(6):
    A[i] = sp.Matrix([[sp.cos(theta[i]), -sp.sin(theta[i])*sp.cos(alpha[i]), sp.sin(theta[i])*sp.sin(alpha[i]), a[i]*sp.cos(theta[i])],
                        [sp.sin(theta[i]), sp.cos(theta[i])*sp.cos(alpha[i]), -sp.cos(theta[i])*sp.sin(alpha[i]), a[i]*sp.sin(theta[i])],
                        [0, sp.sin(alpha[i]), sp.cos(alpha[i]), d[i]],
                        [0, 0, 0, 1]])

# subs

A[0] = A[0].subs({a[0]:0, alpha[0]:-sp.pi/2})
A[1] = A[1].subs({a[1]:0, alpha[1]:sp.pi/2})
A[2] = A[2].subs({theta[2]:0, a[2]:0, alpha[2]:0})
A[3] = A[3].subs({d[3]:0, a[3]:0, alpha[3]:-sp.pi/2})
A[4] = A[4].subs({a[4]:0, alpha[4]:sp.pi/2})
A[5] = A[5].subs({d[5]:0, a[5]:0, alpha[5]:0})

T = A[0]*A[1]*A[2]*A[3]*A[4]*A[5]

print(sp.simplify(T))
