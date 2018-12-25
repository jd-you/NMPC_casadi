function plot_succeed = plot_gif(sol_x, delay)
        PAR = parameters;

    x_plot = 0;
    y_plot = 0;
    x1_plot = 0;
    y1_plot = 0;

    p = plot(x_plot,y_plot,'o', x1_plot, y1_plot, 'o',...
       'EraseMode','background','MarkerSize',5);
    % p2 = plot(x1_plot,y1_plot,'o',...
    %    'EraseMode','background','MarkerSize',5);

    axis([-1.5 1.5 -1.5 1.5]);
    grid on;
    for i = 1:length(sol_x)
        x_plot = PAR.l1 * cos(sol_x(1,i)) + PAR.l2 * cos(sol_x(1,i) + sol_x(3,i));
        y_plot = PAR.l1 * sin(sol_x(1,i)) + PAR.l2 * sin(sol_x(1,i) + sol_x(3,i));
        x1_plot = PAR.l1 * cos(sol_x(1,i));
        y1_plot = PAR.l1 * sin(sol_x(1,i));

        a = line([0 x1_plot], [0 y1_plot]);
        b = line([x1_plot x_plot], [y1_plot y_plot]);
        set(p,'XData',[x_plot;x1_plot],'YData',[y_plot;y1_plot]);


        drawnow;
        pause(delay);
        if (i ~= 101)
            delete(a);
            delete(b);
        end
    end
end