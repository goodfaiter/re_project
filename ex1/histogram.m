for i = 1:50
    e(i) = run();
end
edges = [0:0.1:3];
h = histogram(e,edges);
