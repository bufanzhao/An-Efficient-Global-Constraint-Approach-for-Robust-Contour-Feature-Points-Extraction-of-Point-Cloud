%M-file ,boundary_extract.m
%input variable: point cloud,input_pnts(nx3); number of neighbor points
function [boundary_pnts] = boundary_extract(input_pnts,number_of_neighbor)
n=size(input_pnts,1);     
k=number_of_neighbor;
neighbor_idx=knnsearch(input_pnts,input_pnts,'k',number_of_neighbor+1);  %The number of rows of number_of_neighbor+1 adjacent points corresponding to object point P
neighbor_idx=neighbor_idx(:,2:number_of_neighbor+1);  %The number of rows of number_of_neighbor adjacent points corresponding to object point P
normal_pnts=zeros(3,n);    % Produces an all-zero matrix of 3 by n
line=zeros(n,1);
for i=1:n
    neighbor_pnts=input_pnts(neighbor_idx(i,:),:);  %Obtain the coordinates of the number_of_neighbor points £¨k*3£©
    mean_neighbor=mean(neighbor_pnts,1);    
    v=repmat(mean_neighbor',1,number_of_neighbor)-neighbor_pnts';
      C=v*v';       %3*3 matrix
  [V,D] = eig(C);    %V is the eigenvector matrix corresponding to the eigenvalues£¬D is the eigenvalue diagonal matrix
  [s,j] = min(diag(D));  
                          %s is the eigenvector corresponding to the minimum eigenvalue
  normal_pnts = V(:, j);  %normal vector
 lamda=(repmat(input_pnts(i,:)*normal_pnts,number_of_neighbor,1)-neighbor_pnts*normal_pnts)/(sum(normal_pnts.^2));% kx1 matrix
 proj_pnts=normal_pnts*lamda'+neighbor_pnts';%3xk
 x_unit_vector=(proj_pnts(:,k)'-input_pnts(i,:))/norm(proj_pnts(:,k)'-input_pnts(i,:));% The X-axis normal vector is constructed from the farthest point of the object point and the projected neighboring point 1x3 
 y_unit_vector=cross(normal_pnts,x_unit_vector)/norm(cross(normal_pnts,x_unit_vector)); %unit vector 1x3
 x_plane=sum(repmat(x_unit_vector,k,1).*(proj_pnts'-repmat(input_pnts(i,:),k,1)),2); % 
 y_plane=sum(repmat(y_unit_vector,k,1).*(proj_pnts'-repmat(input_pnts(i,:),k,1)),2); % 
 azimuth= rad2deg(atan2(y_plane,x_plane)); %azimuth
 azimuth(azimuth<0)= azimuth(azimuth<0)+360 ; %negative azimuth changed into positive azimuth
sort_azimuth=sort(azimuth);
diff_azimuth=diff(sort_azimuth);
max_diff(i,:)=max(diff_azimuth);
end
idx=find(max_diff>120);
boundary_pnts=input_pnts(idx,:);

