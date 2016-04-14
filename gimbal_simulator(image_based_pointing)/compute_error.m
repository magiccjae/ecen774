function error = compute_error(in,P)

    %initialize size of output
    error = zeros(2*P.num_agents,1);

    NN = 0;
    for i = 1:P.num_agents,
        est_x(i) = in(NN+1);
        est_y(i) = in(NN+2);
        %est_z(i) = in(NN+3);
        NN = NN + 3;
    end
    true_x = in(NN+1);
    true_y = in(NN+2);
    %true_z = in(NN+3);

    %compute errors in north(x dir) and east(y dir)
    NN = 0;
    for i = 1:P.num_agents,
        error(NN+1:NN+2) = [est_x(i)-true_x; est_y(i)-true_y];
        NN = NN + 2;
    end
end