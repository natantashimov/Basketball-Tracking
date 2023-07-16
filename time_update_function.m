% x_time_update
function x = time_update_function(state_vec,input,alpha,dt) 
pos_now = state_vec(1:2);
vel_now = state_vec(3:4);
beta_now = state_vec(5);
g = input(2);
v = norm(vel_now);
acc_temp = beta_now*alpha*v*vel_now+[0;g];
vel_new = vel_now+acc_temp*dt;
pos_new = pos_now+(vel_new+vel_now)/2*dt;
pos_now = pos_new;
vel_now = vel_new;
beta_new = beta_now; 
x = [pos_new;vel_new;beta_new];
