%Program creates an animation from the .mat files created at the end of
%the main program for a vertical motion maneuver. 
%Uncomment the instructions at the end of the program to create this file.
%
% NOTE: Set saveAnimation to 1 to save animation to AVI file and set
% slowDown to 1 to slow down the animation by a factor given by
% slow_factor.

close all
clear all
clc

load('Quadcopter_UAV_vertical_motion_and_flip.mat')
name1 = "Quadcopter_UAV_vertical_motion_and_flip_zoomed";
name2 = "Quadcopter_UAV_vertical_motion_and_flip";
% name1 = "Quadcopter_UAV_vertical_motion_and_flip_zoomed_slow";
% name2 = "Quadcopter_UAV_vertical_motion_and_flip_slow";

saveAnimation = 0.0;  %Set to 1 to save animation on AVI file.
slowDown = 0.0;
slow_factor = 5;
L = 0.2; %Quadcopter arms length
fps = 60; %Frame per second
Tmax = ceil(max(tt)*fps)/fps;
if(slowDown >= 0.5)
    tt_Vid = 0:(1/fps/slow_factor):Tmax;
else
    tt_Vid = 0:(1/fps):Tmax;
end
tt_Vid(end) = tt(end);
anim_steps = length(tt_Vid);
Xpos_Vid = interp1(tt,px,tt_Vid,'makima');
Zpos_Vid = interp1(tt,pz,tt_Vid,'makima');
ang_Vid = interp1(tt,ang,tt_Vid,'makima');

Xpos_Vid(end) = px(end);
Zpos_Vid(end) = pz(end);
ang_Vid(end) = ang(end);
ang_Vid = -ang_Vid;

%default_color_map = get(gca,'colormap');
%color_array = GetColorArray(tt_Vid,default_color_map);
color_array = GetColorArray(tt_Vid,colormap('jet'));

if saveAnimation >= 0.5
    v1 = VideoWriter(name1,'Motion JPEG AVI');
    v1.Quality = 100;
    v1.FrameRate = fps;
    open(v1);
    v2 = VideoWriter(name2,'Motion JPEG AVI');
    v2.Quality = 100;
    v2.FrameRate = fps;
    open(v2);
end

figure(1)
set(gcf, 'color', [1 1 1])
set(gcf, 'position',[100,100,600,500])
ax = gca;
ax.FontSize = 17; 
set(gca,'TickLabelInterpreter','latex')

cc = colorbar("eastoutside",...
                'Ticks',[0 1],...
                'TickLabels',{'0', '$T_{\rm fin}$'}, ...          
                'Fontsize', 17, 'TickLabelInterpreter', 'latex');
    cc.Label.String = "$t$ (s)";
    cc.Label.Interpreter = 'latex';

grid on
box on
     
for kk = 1:anim_steps
    
    hold on
    plot([Xpos_Vid(kk)-L*cos(ang_Vid(kk)) Xpos_Vid(kk)+L*cos(ang_Vid(kk))], [Zpos_Vid(kk)-L*sin(ang_Vid(kk)) Zpos_Vid(kk)+L*sin(ang_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
    plot([Xpos_Vid(kk)-L*cos(ang_Vid(kk)) Xpos_Vid(kk)-L*cos(ang_Vid(kk))-0.25*L*sin(ang_Vid(kk))], [Zpos_Vid(kk)-L*sin(ang_Vid(kk)) Zpos_Vid(kk)-L*sin(ang_Vid(kk))+0.25*L*cos(ang_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
    plot([Xpos_Vid(kk)+L*cos(ang_Vid(kk)) Xpos_Vid(kk)+L*cos(ang_Vid(kk))-0.25*L*sin(ang_Vid(kk))], [Zpos_Vid(kk)+L*sin(ang_Vid(kk)) Zpos_Vid(kk)+L*sin(ang_Vid(kk))+0.25*L*cos(ang_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
    hold off

    xlim([-1+Xpos_Vid(kk) 1+Xpos_Vid(kk)])
    xlabel('$x$ (m)','Interpreter','latex','FontSize',17)
    
    ylim([-1+Zpos_Vid(kk) 1+Zpos_Vid(kk)])
    ylabel('$y$ (m)','Interpreter','latex','FontSize',17)
        
    rrTxt = sprintf('$t = %.2f$ s',tt_Vid(kk));
    rr = text(Xpos_Vid(kk)+0.4, Zpos_Vid(kk)+0.75, rrTxt, 'Interpreter', 'latex', 'FontSize', 17);

    set(gca,'position',[0.125,0.125,0.68,0.8])

    pause(1/fps)
    if saveAnimation >= 0.5
        frame = getframe(gcf);
        writeVideo(v1,frame);
    end
    set(rr,'Visible','off')
end

figure(2)
set(gcf, 'color', [1 1 1])
set(gcf, 'position',[100,100,500,800])
set(gcf, 'colormap',colormap('jet'))
ax = gca;
ax.FontSize = 17; 
set(gca,'TickLabelInterpreter','latex')

cc = colorbar("eastoutside",...
                'Ticks',[0 1],...
                'TickLabels',{'0', '$T_{\rm fin}$'}, ...          
                'Fontsize', 17, 'TickLabelInterpreter', 'latex');
    cc.Label.String = "$t$ (s)";
    cc.Label.Interpreter = 'latex';

xlim([-1.25 1.25])
xlabel('$x$ (m)','Interpreter','latex','FontSize',17)
    
ylim([-0.25 3.25])
ylabel('$y$ (m)','Interpreter','latex','FontSize',17)

grid on
box on
     
for kk = 1:anim_steps
    
    hold on
    plot([Xpos_Vid(kk)-L*cos(ang_Vid(kk)) Xpos_Vid(kk)+L*cos(ang_Vid(kk))], [Zpos_Vid(kk)-L*sin(ang_Vid(kk)) Zpos_Vid(kk)+L*sin(ang_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
    plot([Xpos_Vid(kk)-L*cos(ang_Vid(kk)) Xpos_Vid(kk)-L*cos(ang_Vid(kk))-0.25*L*sin(ang_Vid(kk))], [Zpos_Vid(kk)-L*sin(ang_Vid(kk)) Zpos_Vid(kk)-L*sin(ang_Vid(kk))+0.25*L*cos(ang_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
    plot([Xpos_Vid(kk)+L*cos(ang_Vid(kk)) Xpos_Vid(kk)+L*cos(ang_Vid(kk))-0.25*L*sin(ang_Vid(kk))], [Zpos_Vid(kk)+L*sin(ang_Vid(kk)) Zpos_Vid(kk)+L*sin(ang_Vid(kk))+0.25*L*cos(ang_Vid(kk))],'LineWidth',2,'Color',color_array(kk,:))
    hold off
        
    rrTxt = sprintf('$t = %.2f$ s',tt_Vid(kk));
    rr = text(0.3, 3.15, rrTxt, 'Interpreter', 'latex', 'FontSize', 17);

    set(gca,'position',[0.168,0.11,0.59,0.86])

    pause(1/fps)
    if saveAnimation >= 0.5
        frame = getframe(gcf);
        writeVideo(v2,frame);
    end
    set(rr,'Visible','off')
end

if saveAnimation >= 0.5
    close(v1);
    close(v2)
end

%% 
function color_array = GetColorArray(x_array,default_color_map)
x_fake = linspace(min(x_array),max(x_array),size(default_color_map,1));
color_array = interp1(x_fake, default_color_map, x_array);
end