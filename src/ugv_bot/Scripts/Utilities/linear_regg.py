import numpy as np
from fractions import Fraction

if __name__ == '__main__':

  #enter coordinates vectors
  Y = np.array([[-420,-330]]).T
  X = np.array([[300,0]]).T

  # y =mx +c 

  O = np.ones(X.shape)
  A = np.append(X,O,axis=1)

  A_t = A.T

  A_t_dot_A = A_t.dot(A)

  A_t_dot_A_inv = np.linalg.inv(A_t_dot_A)

  A_t_dot_A_inv_dot_A_t = A_t_dot_A_inv.dot(A_t)

  ans = A_t_dot_A_inv_dot_A_t.dot(Y)

  m = float(ans[0])
  c = float(ans[1])

  #print(type(m))
  #print(m)

  simple_m = Fraction(m).limit_denominator()
  simple_c = Fraction(c).limit_denominator()

  #np.append(x, y, axis=1)
  print("slope m =",simple_m)
  print("intercept c =",simple_c)



