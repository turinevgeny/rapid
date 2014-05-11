arr = csvread('rvec_and_tvec.txt');

rx = arr( : , 1 );
ry = arr( : , 2 );
rz = arr( : , 3 );

tx = arr( : , 4 );
ty = arr( : , 5 );
tz = arr( : , 6 );

figure('name','Delta rotation vectors')
scatter3(rx,ry,rz,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[.75 .75 0])
view(-30,10)

figure('name','Delta translation vectors')
scatter3(tx,ty,tz,...
        'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75])
view(-30,10)