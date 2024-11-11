import sympy as sp


# transformation matrix


# states
states = sp.Matrix([
                [0.210182586662170, -0.140623923147677, -1.66990262059900, 0.239730216951784],
                [-0.00957088854224075, -0.0804074916976769, -1.39056517891334, -0.0998236561838834],
                [-0.209361009920126, -0.148174627959482, -1.69310730868183, 0.270485609846419],
                [0.0286286027851985, -0.248365484092256, -1.91392240195310, 0.591491559250461],
                [0.210182586662170, -0.140623923147677, -1.66990262059900, 0.239730216951784]
                ])

Jacobi_0 = sp.Matrix([[-0.032, -0.068, 0.022, 0, 0],
                    [0.15, -0.015, 0.005, 0, 0],
                    [0, 0.153, 0.14, 0.05, 0.05],
                    [0, 0.209, 0.209, 0.209, 0.209],
                    [0, -0.978, -0.978, -0.978, -0.978],
                    [1, 0, 0, 0, 0]])

Jacobi_90 = sp.Matrix([[0.001, -0.102, -0.009, 0, 0],
                    [0.15, 0.001, 0, 0, 0],
                    [0, 0.15, 0.143, 0.05, 0.05],
                    [0, -0.01, -0.01, -0.01, -0.01],
                    [0, -1.0, -1.0, -1.0, -1.0],
                    [1, 0, 0, 0, 0]])

Jacobi_180 = sp.Matrix([[0.032, -0.066, 0.024, 0, 0],
                        [0.15, 0.014, -0.005, 0, 0],
                        [0, 0.153, 0.14, 0.05, 0.05],
                        [0, -0.208, -0.208, -0.208, -0.208],
                        [0, -0.978, -0.978, -0.978, -0.978],
                        [1, 0, 0, 0, 0]])

Jacobi_270 = sp.Matrix([[-0.004, -0.038, 0.052, 0, 0],
                       [0.15, -0.001, 0.001, 0, 0],
                       [0, 0.15, 0.127, 0.05, 0.05],
                       [0, 0.029, 0.029, 0.029, 0.029],
                       [0, -1.0, -1.0, -1.0, -1.0],
                       [1, 0, 0, 0, 0]])

Jacobi_360 = Jacobi_0

Jacobi_0 = Jacobi_0[:3, :4]
Jacobi_90 = Jacobi_90[:3, :4]
Jacobi_180 = Jacobi_180[:3, :4]
Jacobi_270 = Jacobi_270[:3, :4]
Jacobi_360 = Jacobi_360[:3, :4]

t = sp.symbols('t')

# defining the polynomial constants as variables
def create_symbols(prefix):
    return sp.Matrix([[sp.symbols(f'{prefix}{i}{j}') for j in range(5, -1, -1)] for i in range(1, 5)])

A = create_symbols('A')
B = create_symbols('B')
C = create_symbols('C')
D = create_symbols('D')

# defining polynomial constants for the segments:
t_vec = sp.Matrix([t**i for i in range(5, -1, -1)])
t_vec_diff = sp.Matrix([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0])
t_vec_diffdiff = sp.Matrix([20*t**3, 12*t**2, 6*t, 2, 0, 0])

# defining equations stating boundary conditions

# angular position boundaries
eq1 = states[0,:].T - (A @ t_vec.subs(t, 0))
eq2 = states[1,:].T - (A @ t_vec.subs(t, 2))
eq3 = states[1,:].T - (B @ t_vec.subs(t, 0))
eq4 = states[2,:].T - (B @ t_vec.subs(t, 2))
eq5 = states[2,:].T - (C @ t_vec.subs(t, 0))
eq6 = states[3,:].T - (C @ t_vec.subs(t, 2))
eq7 = states[3,:].T - (D @ t_vec.subs(t, 0))
eq8 = states[4,:].T - (D @ t_vec.subs(t, 2))

# velocity boundaries - using jacobian
v0  = sp.Matrix([0, 0, 0])
v9  = sp.Matrix([0, -0.27, 0])
v18 = sp.Matrix([0, 0, -0.27])
v27 = sp.Matrix([0, 27, 0])
v36 = sp.Matrix([0, 0, 0])

# since the arm is overconstrained, we need to use the pseudoinverse:
Jacobi_0_pseudoinv = Jacobi_0.T @ (Jacobi_0 @ Jacobi_0.T).inv()
Jacobi_90_pseudoinv = Jacobi_90.T @ (Jacobi_90 @ Jacobi_90.T).inv()
Jacobi_180_pseudoinv = Jacobi_180.T @ (Jacobi_180 @ Jacobi_180.T).inv()
Jacobi_270_pseudoinv = Jacobi_270.T @ (Jacobi_270 @ Jacobi_270.T).inv()
Jacobi_360_pseudoinv = Jacobi_360.T @ (Jacobi_360 @ Jacobi_360.T).inv()

eq9  = (A @ t_vec_diff.subs(t, 0)) - Jacobi_0_pseudoinv @ v0 
eq10 = (A @ t_vec_diff.subs(t, 2)) - Jacobi_90_pseudoinv @ v9
eq11 = (B @ t_vec_diff.subs(t, 0)) - Jacobi_90_pseudoinv @ v9
eq12 = (B @ t_vec_diff.subs(t, 2)) - Jacobi_180_pseudoinv @ v18
eq13 = (C @ t_vec_diff.subs(t, 0)) - Jacobi_180_pseudoinv @ v18
eq14 = (C @ t_vec_diff.subs(t, 2)) - Jacobi_270_pseudoinv @ v27
eq15 = (D @ t_vec_diff.subs(t, 0)) - Jacobi_270_pseudoinv @ v27
eq16 = (D @ t_vec_diff.subs(t, 2)) - Jacobi_360_pseudoinv @ v36

# angular acceleration boundaries
eq17 = (A @ t_vec_diffdiff.subs(t, 0)) - sp.Matrix([0, 0, 0, 0])
eq18 = (A @ t_vec_diffdiff.subs(t, 2)) - sp.Matrix([0, 0, 0, 0])
eq19 = (B @ t_vec_diffdiff.subs(t, 0)) - sp.Matrix([0, 0, 0, 0])
eq20 = (B @ t_vec_diffdiff.subs(t, 2)) - sp.Matrix([0, 0, 0, 0])
eq21 = (C @ t_vec_diffdiff.subs(t, 0)) - sp.Matrix([0, 0, 0, 0])
eq22 = (C @ t_vec_diffdiff.subs(t, 2)) - sp.Matrix([0, 0, 0, 0])
eq23 = (D @ t_vec_diffdiff.subs(t, 0)) - sp.Matrix([0, 0, 0, 0])
eq24 = (D @ t_vec_diffdiff.subs(t, 2)) - sp.Matrix([0, 0, 0, 0])

# flatten all the equations
for i in range(1, 25):
    exec(f"eq{i} = sp.flatten(eq{i})")

# solving the system of equations
eqs = [eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8, eq9, eq10, eq11, eq12, eq13, eq14, eq15, eq16, eq17, eq18, eq19, eq20, eq21, eq22, eq23, eq24]

# the eqs list has lists of equations inside, we need to flatten it
eqs = [eq for sublist in eqs for eq in sublist]

# Flatten the matrices into vectors
variables = sp.flatten(A) + sp.flatten(B) + sp.flatten(C) + sp.flatten(D)

# Solve for the individual elements
sol = sp.solve(eqs, variables)

# # Use numerical solver with initial guesses
# sol2 = sp.nsolve(eqs, variables, [10]*len(variables))

# # printing the numerical solution
# sp.pprint(sol2)

# printing solution with 4 decimal places
for var, value in sol.items():
    print(f"{var} = {value.evalf(4)}")

# assigning the solution to the matrices
A_sol = sp.Matrix([[sol[sp.symbols(f'A{i}{j}')] for j in range(5, -1, -1)] for i in range(1, 5)])
B_sol = sp.Matrix([[sol[sp.symbols(f'B{i}{j}')] for j in range(5, -1, -1)] for i in range(1, 5)])
C_sol = sp.Matrix([[sol[sp.symbols(f'C{i}{j}')] for j in range(5, -1, -1)] for i in range(1, 5)])
D_sol = sp.Matrix([[sol[sp.symbols(f'D{i}{j}')] for j in range(5, -1, -1)] for i in range(1, 5)])

# printing the solution with 4 decimal places
print("A:")
sp.pprint(A_sol.evalf(4))
print("B:")
sp.pprint(B_sol.evalf(4))
print("C:")
sp.pprint(C_sol.evalf(4))
print("D:")
sp.pprint(D_sol.evalf(4))

# make expressions for the polynomials

# segment A
q_A = A_sol @ t_vec
q_B = B_sol @ t_vec
q_C = C_sol @ t_vec
q_D = D_sol @ t_vec

# printing the expressions
print("q_A:")
sp.pprint(q_A.evalf(4))
print("q_B:")
sp.pprint(q_B.evalf(4))
print("q_C:")
sp.pprint(q_C.evalf(4))
print("q_D:")
sp.pprint(q_D.evalf(4))



