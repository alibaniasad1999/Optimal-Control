for i = 1:17
    x(i)
    y(i)
    y(i) * cos(x(i) + y(i)) + sin(x(i) + y(i)) + x(i) * cos(x(i) - y(i))
end