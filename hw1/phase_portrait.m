function phase_portrait(f, left_bound, right_bound, num_points)

x1 = linspace(left_bound, right_bound, num_points);
x2 = linspace(left_bound, right_bound, num_points);

[x,y] = meshgrid(x1,x2);
size(x);
size(y);

u = zeros(size(x));
v = zeros(size(x));

t = 0;
for i = 1:numel(x)
    Xprime = f(t,[x(i); y(i)]);
    u(i) = Xprime(1);
    v(i) = Xprime(2);
end

figure;
quiver(x,y,u,v,'r'); 
xlabel('x_1');
ylabel('x_2');
axis ([left_bound right_bound left_bound right_bound]);

hold on;
% for x20 = [0 1 2 3 4]
    for x20 = x2
        for x10 = x1
            if (x20 == left_bound || x20 == right_bound || x10 == left_bound || x10 == right_bound)
                [ts,xs] = ode45(f,[0,50],[x10;x20]);
                plot(xs(:,1),xs(:,2))
                plot(xs(1,1),xs(1,2),'bo')
                plot(xs(end,1),xs(end,2),'ks')
            end
        end
    end
% end
hold off;