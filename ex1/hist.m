for i = 1:50
    e(i) = run();
end
edges = [0:0.1:6];
histogram(e,edges);
