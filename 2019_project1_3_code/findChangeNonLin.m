function [indOfVertices]= findChangeNonLin(path)

% path=[0,0,0;1,0,0;2,0,0;3,0,0;4,1,2;4,2,4;4,3,6];
% path=[0,0,0;1,0,0;2,0,0;3,0,0;3,1,1;3,2,2;3,3,3;3,4,4];


dPath1=diff(path);
dPath2=dPath1(2:end,:);
dPath1=dPath1(1:end-1,:);
mDpath=sqrt(dot(dPath1,dPath1,2));
mDpath1=sqrt(dot(dPath2,dPath2,2));
cosA1_2=dot(dPath1,dPath2,2)./(mDpath.*mDpath1);

indOfVertices=find(cosA1_2<0.99)+1;
% scatter3(path(:,1),path(:,2),path(:,3))
end