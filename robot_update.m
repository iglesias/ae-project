function robots = robot_update(robots, Rrobo, z, l, m)

global DEBLV

% Useful constants
nrobots = length(robots);

robots = predict_cross_terms(robots);

for r = 1:nrobots

  if robots(r).never_updated

    % Just update if r is either the observer or the observed
    if r == l

      if DEBLV
        fprintf('>>>>>>>> Update_init_observer robot %d\n', r);
      end
      robots(r) = update_init_observer(robots(m), robots(l), Rrobo, z);

    elseif r == m

      if DEBLV
        fprintf('>>>>>>>> Update_init_observed robot %d\n', r);
      end
      robots(r) = update_init_observed(robots(m), robots(l), Rrobo, z);

    end

  else

    if DEBLV
      fprintf('>>>>>>>> Update robot %d\n', r);
    end
    robots(r) = update(robots, r, l, m, Rrobo, z);

  end

end

end

