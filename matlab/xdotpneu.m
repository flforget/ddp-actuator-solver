function xdot = xdotpneu(x,u1,u2)

xdot = jointdynamics4state(u1,u2,x)
