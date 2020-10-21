function Nu = S_Wrap(alpha)

    Nu = alpha;

    while (Nu > pi)
        Nu =Nu - 2 * pi;
    end

    while (Nu <= -pi)
        Nu = Nu + 2 * pi;
    end
    
end