function [map, start, goal, res, xMax, yMax] = getMap(plot)
  I = imread('IGVCmap.jpg');
  map = im2bw(I, 0.7); % Convert to 0-1 image
  map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
  [xMax,yMax]= size(map); % Map size

  % Robot start position
  res = 0.1;
  start = [40 5 pi];

  % Target location
  goal = [50 10];

  % Plotting
  if plot
    figure(1); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    plot(start(1)/res, start(2)/res, 'ro', 'MarkerSize',10, 'LineWidth', 3);
    plot(goal(1)/res, goal(2)/res, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
    axis equal
  end
end