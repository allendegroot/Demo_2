% Rho (dot) = radius * (theta1 (dot) * theta2 (dot))/2
out = sim('Grho_transferFn_ID');


% Experimental time and velocity defined in workspace



figure(1);
plot(out.lin_velocity)
hold on
plot(experimentalTime, experimentalVelocity)
title('Linear Velocity')
xlim([0 4]);
xlabel('Time (seconds)')
ylabel('Linear Velocity (m/s)')