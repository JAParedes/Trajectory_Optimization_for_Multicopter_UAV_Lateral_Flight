% Program creates an animation from the .mat files created at the end of
% the main program for a vertical displacement with flip maneuver. 
%
% Two animations are created. The first one is zoomed in on the multicopter
% and the second one is zoomed out to watch theh trajectory in greater
% detail.
%
% NOTE: Set saveAnimation to 1 to save animation to AVI file and set
% slowDown to 1 to slow down the animation by a factor given by
% slow_factor in lines 16 and 19, respectively.

close all
clear all
clc

%% Rotational dynamics were considered in the trajectory optimization procedure (only used for video file naming purposes)
rot_dyn = 0;    % 0: Dynamics do not consider rotational dynamics, 1: Dynamics consider rotational dynamics

%% Save animationn video option
saveAnimation = 0;  % 0: Do not save video file, 1: Save video file

%% Slow down option
slowDown = 0;
slow_factor = 5; % Slow down factor, play back speed is given by 1/slow_factor

%% Animation setup

if rot_dyn == 0
    load('Multicopter_vertical_motion_and_flip.mat')
elseif rot_dyn == 1
    load('Multicopter_w_Rot_Dyn_vertical_motion_and_flip.mat')
end

L = 0.2;                        % Multicopter arms length
fps = 60;                       % Frame per second
Tmax = ceil(max(tt)*fps)/fps;   % Obtain a multiple of the chosen fps as the maximum time

if rot_dyn == 0
    if slowDown == 0
        tt_Vid = 0:(1/fps):Tmax;
        name1 = "Multicopter_vertical_motion_and_flip_zoomed";
        name2 = "Multicopter_vertical_motion_and_flip";
    else
        tt_Vid = 0:(1/fps/slow_factor):Tmax;
        name1 = "Multicopter_vertical_motion_and_flip_zoomed_slow";
        name2 = "Multicopter_vertical_motion_and_flip_slow";
    end
elseif rot_dyn == 1
    if slowDown == 0
        tt_Vid = 0:(1/fps):Tmax;
        name1 = "Multicopter_w_Rot_Dyn_vertical_motion_and_flip_zoomed";
        name2 = "Multicopter_w_Rot_Dyn_vertical_motion_and_flip";
    else
        tt_Vid = 0:(1/fps/slow_factor):Tmax;
        name1 = "Multicopter_w_Rot_Dyn_vertical_motion_and_flip_zoomed_slow";
        name2 = "Multicopter_w_Rot_Dyn_vertical_motion_and_flip_slow";
    end
end

tt_Vid(end) = tt(end);
anim_steps = length(tt_Vid);
% Animation data is obtained by interpolating values from the obtained
% optimal trajectories at each time in the animation time vector
Xpos_Vid = interp1(tt,px,tt_Vid,'makima');
Zpos_Vid = interp1(tt,pz,tt_Vid,'makima');
theta_Vid = interp1(tt,theta,tt_Vid,'makima');

Xpos_Vid(end) = px(end);
Zpos_Vid(end) = pz(end);
theta_Vid(end) = theta(end);
theta_Vid = -theta_Vid;         % Adjusting for appropriate plotting

color_array = GetColorArray(tt_Vid,colormap('jet'));

% Setting up video saving parameters
if saveAnimation == 1
    v1 = VideoWriter(name1,'Motion JPEG AVI');
    v1.Quality = 100;
    v1.FrameRate = fps;
    open(v1);
    v2 = VideoWriter(name2,'Motion JPEG AVI');
    v2.Quality = 100;
    v2.FrameRate = fps;
    open(v2);
end

%% Animation of the multicopter trajectory zoomed in on the multicopter

figure(1)
set(gcf, 'color', [1 1 1])
set(gcf, 'position',[100,100,600,500])
ax = gca;
ax.FontSize = 17; 
set(gca,'TickLabelInterpreter','latex')

cc = colorbar("eastoutside",...
                'Ticks',[0 1],...
                'TickLabels',{'0', '$T$'}, ...          
                'Fontsize', 17, 'TickLabelInterpreter', 'latex');
    cc.Label.String = "$t$ (s)";
    cc.Label.Interpreter = 'latex';

grid on
box on
     
for kk = 1:anim_steps
    
    if (rem(kk-1,slow_factor)==0 || slowDown~=1)
        hold on
        plot([Xpos_Vid(kk)-L*cos(theta_Vid(kk)) Xpos_Vid(kk)+L*cos(theta_Vid(kk))], [Zpos_Vid(kk)-L*sin(theta_Vid(kk)) Zpos_Vid(kk)+L*sin(theta_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
        plot([Xpos_Vid(kk)-L*cos(theta_Vid(kk)) Xpos_Vid(kk)-L*cos(theta_Vid(kk))-0.25*L*sin(theta_Vid(kk))], [Zpos_Vid(kk)-L*sin(theta_Vid(kk)) Zpos_Vid(kk)-L*sin(theta_Vid(kk))+0.25*L*cos(theta_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
        plot([Xpos_Vid(kk)+L*cos(theta_Vid(kk)) Xpos_Vid(kk)+L*cos(theta_Vid(kk))-0.25*L*sin(theta_Vid(kk))], [Zpos_Vid(kk)+L*sin(theta_Vid(kk)) Zpos_Vid(kk)+L*sin(theta_Vid(kk))+0.25*L*cos(theta_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
        if kk > 1
            plot(Xpos_Vid(1:kk), Zpos_Vid(1:kk),'LineWidth',2,'Color','k')
        end
        multicopterq1 = plot([Xpos_Vid(kk)-L*cos(theta_Vid(kk)) Xpos_Vid(kk)+L*cos(theta_Vid(kk))], [Zpos_Vid(kk)-L*sin(theta_Vid(kk)) Zpos_Vid(kk)+L*sin(theta_Vid(kk))],'LineWidth',2,'Color','k');
        multicopterq2 = plot([Xpos_Vid(kk)-L*cos(theta_Vid(kk)) Xpos_Vid(kk)-L*cos(theta_Vid(kk))-0.25*L*sin(theta_Vid(kk))], [Zpos_Vid(kk)-L*sin(theta_Vid(kk)) Zpos_Vid(kk)-L*sin(theta_Vid(kk))+0.25*L*cos(theta_Vid(kk))],'LineWidth',2,'Color','k');
        multicopterq3 = plot([Xpos_Vid(kk)+L*cos(theta_Vid(kk)) Xpos_Vid(kk)+L*cos(theta_Vid(kk))-0.25*L*sin(theta_Vid(kk))], [Zpos_Vid(kk)+L*sin(theta_Vid(kk)) Zpos_Vid(kk)+L*sin(theta_Vid(kk))+0.25*L*cos(theta_Vid(kk))],'LineWidth',2,'Color','k');
        hold off
    end

    xlim([-1+Xpos_Vid(kk) 1+Xpos_Vid(kk)])
    xlabel('$x$ (m)','Interpreter','latex','FontSize',17)
    
    ylim([-1+Zpos_Vid(kk) 1+Zpos_Vid(kk)])
    ylabel('$z$ (m)','Interpreter','latex','FontSize',17)
        
    rrTxt = sprintf('$t = %.2f$ s',tt_Vid(kk));
    rr = text(Xpos_Vid(kk)+0.4, Zpos_Vid(kk)+0.75, rrTxt, 'Interpreter', 'latex', 'FontSize', 17);

    set(gca,'position',[0.125,0.125,0.68,0.8])

    pause(1/fps)
    if saveAnimation == 1
        frame = getframe(gcf);
        writeVideo(v1,frame);
    end
    set(rr,'Visible','off')
    set(multicopterq1,'Visible','off')
    set(multicopterq2,'Visible','off')
    set(multicopterq3,'Visible','off')
end

%% Animation of the multicopter trajectory zoomed out to watch the entire trajectory

figure(2)
set(gcf, 'color', [1 1 1])
set(gcf, 'position',[100,100,500,800])
set(gcf, 'colormap',colormap('jet'))
ax = gca;
ax.FontSize = 17; 
set(gca,'TickLabelInterpreter','latex')

cc = colorbar("eastoutside",...
                'Ticks',[0 1],...
                'TickLabels',{'0', '$T$'}, ...          
                'Fontsize', 17, 'TickLabelInterpreter', 'latex');
    cc.Label.String = "$t$ (s)";
    cc.Label.Interpreter = 'latex';

xlim([-1.25 1.25])
xlabel('$x$ (m)','Interpreter','latex','FontSize',17)
    
ylim([-0.25 3.25])
ylabel('$z$ (m)','Interpreter','latex','FontSize',17)

grid on
box on
     
for kk = 1:anim_steps
    
    if (rem(kk-1,slow_factor)==0 || slowDown~=1)
        hold on
        plot([Xpos_Vid(kk)-L*cos(theta_Vid(kk)) Xpos_Vid(kk)+L*cos(theta_Vid(kk))], [Zpos_Vid(kk)-L*sin(theta_Vid(kk)) Zpos_Vid(kk)+L*sin(theta_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
        plot([Xpos_Vid(kk)-L*cos(theta_Vid(kk)) Xpos_Vid(kk)-L*cos(theta_Vid(kk))-0.25*L*sin(theta_Vid(kk))], [Zpos_Vid(kk)-L*sin(theta_Vid(kk)) Zpos_Vid(kk)-L*sin(theta_Vid(kk))+0.25*L*cos(theta_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
        plot([Xpos_Vid(kk)+L*cos(theta_Vid(kk)) Xpos_Vid(kk)+L*cos(theta_Vid(kk))-0.25*L*sin(theta_Vid(kk))], [Zpos_Vid(kk)+L*sin(theta_Vid(kk)) Zpos_Vid(kk)+L*sin(theta_Vid(kk))+0.25*L*cos(theta_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
        if kk > 1
            plot(Xpos_Vid(1:kk), Zpos_Vid(1:kk),'LineWidth',2,'Color','k')
        end
        multicopterq1 = plot([Xpos_Vid(kk)-L*cos(theta_Vid(kk)) Xpos_Vid(kk)+L*cos(theta_Vid(kk))], [Zpos_Vid(kk)-L*sin(theta_Vid(kk)) Zpos_Vid(kk)+L*sin(theta_Vid(kk))],'LineWidth',2,'Color','k');
        multicopterq2 = plot([Xpos_Vid(kk)-L*cos(theta_Vid(kk)) Xpos_Vid(kk)-L*cos(theta_Vid(kk))-0.25*L*sin(theta_Vid(kk))], [Zpos_Vid(kk)-L*sin(theta_Vid(kk)) Zpos_Vid(kk)-L*sin(theta_Vid(kk))+0.25*L*cos(theta_Vid(kk))],'LineWidth',2,'Color','k');
        multicopterq3 = plot([Xpos_Vid(kk)+L*cos(theta_Vid(kk)) Xpos_Vid(kk)+L*cos(theta_Vid(kk))-0.25*L*sin(theta_Vid(kk))], [Zpos_Vid(kk)+L*sin(theta_Vid(kk)) Zpos_Vid(kk)+L*sin(theta_Vid(kk))+0.25*L*cos(theta_Vid(kk))],'LineWidth',2,'Color','k');
        hold off
    end
        
    rrTxt = sprintf('$t = %.2f$ s',tt_Vid(kk));
    rr = text(0.3, 3.15, rrTxt, 'Interpreter', 'latex', 'FontSize', 17);

    set(gca,'position',[0.168,0.11,0.59,0.86])

    pause(1/fps)
    if saveAnimation == 1
        frame = getframe(gcf);
        writeVideo(v2,frame);
    end
    set(rr,'Visible','off')
    set(multicopterq1,'Visible','off')
    set(multicopterq2,'Visible','off')
    set(multicopterq3,'Visible','off')
end

%% Closing files in case they were opened for animation saving purposes
if saveAnimation == 1
    close(v1);
    close(v2)
end

%% Function for assigning a color from a chosen colormap to each component in a time vector
function color_array = GetColorArray(x_array,default_color_map)
x_fake = linspace(min(x_array),max(x_array),size(default_color_map,1));
color_array = interp1(x_fake, default_color_map, x_array);
end