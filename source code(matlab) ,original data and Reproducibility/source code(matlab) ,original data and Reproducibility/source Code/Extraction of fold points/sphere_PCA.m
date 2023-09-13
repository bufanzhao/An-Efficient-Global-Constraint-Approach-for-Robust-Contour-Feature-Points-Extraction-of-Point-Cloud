
%Total least squares£ºA(x-xj)+B(y-yj)+C(z-zj)=0  Ax+By+Cz+D=0 £¨A,B,C£©vector is the eigenvector corresponding to the maximum eigenvalue
%Input:input_pnts(nx3)  point cloud
%Output:Fitting plane parameters 

function [parameter] = sphere_PCA(input_pnts)
   mean_neighbor=mean(input_pnts,1);
   difference=input_pnts-mean_neighbor;
    M=difference'*difference;
    [V, ~]=svd(M);
    A=V(1,1);B=V(2,1);C=V(3,1); 
    D=mean_neighbor*V(:,1);
    parameter=[A,B,C,D];