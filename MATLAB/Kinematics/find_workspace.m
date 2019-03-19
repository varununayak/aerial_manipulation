function find_workspace()
% -This function find the workspace of the manipulator using the forward
% kinematics
% -The workspace is then plotted on 2-D with the base frame at the origin
% TODO: Add mesh of manipulator along with the drone in the plot for better
% understanding of clearances

hold on;
i=1;

    for t2=0:0.1:2*pi
        for t3=0:0.1:2*pi
            a = fk_v2(0,t2,t3);
             X(i)= a.x;
            Y(i)= a.y;
            Z(i)= a.z;
            i=i+1;
        end
    end

scatter(X,Z,'*');
grid on;
axis square
end

