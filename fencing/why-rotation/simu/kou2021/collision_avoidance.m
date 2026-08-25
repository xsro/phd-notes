function alpha=collision_avoidance(dij,alpha_max,d,mu)
if dij<d
    alpha=alpha_max;
elseif dij<mu
    alpha=1/(dij-d)-1/(mu-d);
    if alpha>alpha_max
        alpha=alpha_max;
    end
else
    alpha=0;
end
end