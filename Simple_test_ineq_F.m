function v = Simple_test_ineq_F(x,c_cost,a_ineq,b_ineq)

F = sin((x-c_cost)/3);
h = -((x(1,1)-a_ineq)^2+(x(2,1)-a_ineq)^2)+b_ineq^2;
v=[F;h];