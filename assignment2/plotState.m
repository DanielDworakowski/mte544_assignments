function [] = plotState(X, newFig, name)
  if (newFig)
    figure;
  end
  plot(X(:,1), X(:,2));
  title('State x vs y position');
  ylabel('y (m)');
  xlabel('x (m)');
  axis equal
  saveas(gcf, strcat(name,'.png'));
end