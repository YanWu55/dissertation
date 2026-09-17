function ts = settling_time(t,errorMeasure,tolerance)
%SETTLING_TIME Earliest time after which error remains within tolerance.

inside = errorMeasure <= tolerance;
ts = NaN;

for k = 1:numel(t)
    if all(inside(k:end))
        ts = t(k);
        return;
    end
end
end
