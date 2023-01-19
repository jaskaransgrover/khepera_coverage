
function display_heatmap(X, Y, Z)

    close all
    iptsetpref('ImshowBorder','tight');
    set(0,'DefaultFigureMenu','none');
    FigHandle = figure2();
    iptsetpref('ImshowBorder','tight');
    set(0,'DefaultFigureMenu','none');
    % figure('units','normalized','outerposition',[0 0 1 1],'color','white')

    x0             = -56.7*2.54*0.01 ;% -20 
    y0             = -37.5*2.54*0.01 ;% -10;
    width          = 152.5*2.54*0.01 ;% 40 
    height         = 80*2.54*0.01 ; % 20 
    rectangle('Position',[x0 y0 width height],'linewidth',6,'edgecolor','red')
    hold on
    
    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset;
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left-0.03 bottom-0.003 ax_width+0.02 ax_height+0.014];
    axis equal

    contourf(X, Y, Z,'LineColor','none')

    xlim([x0,x0+width])
    ylim([y0,y0+height])
    iptsetpref('ImshowBorder','tight');
    set(0,'DefaultFigureMenu','none');
end