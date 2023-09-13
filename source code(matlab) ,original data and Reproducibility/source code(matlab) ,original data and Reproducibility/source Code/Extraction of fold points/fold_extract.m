% Three eigenvectors are obtained according to feature decomposition
% V(:,1)，V(:,2)，V(:,3)。 V(:,1)and V(:,2)construct the plane 
%nomal vector of the plane is V(:,3),  A=V(1,3),B=V(2,3),C=V(3,3), 
%According to normal vector（A,B,C）and object point (x0，y0，z0) to construct the plane equation：A（x-x0）+B（y-y0）+C（z-z0）=0
% Ax+By+Cz-D=0  The points in the bounding ball are projected into plane and transformed into two dimensional point 
% Find the two farthest points and construct the line equation 
%Input: radius(radius of bounding ball),  PL_threshold：The mse threshold  the distance from point to line.   DP_DS: the gap between the number of points on bothe side of line
%rank_dis_threshold:  The points p which correspong to the first few distance are crease points(Generally 3).
%Output: the fold points of object point cloud 


function [fold_pnt] = fold_extract(input,radius,PL_threshold,DP_DS, rank_dis_threshold)    %该方法效果比较好
n=size(input,1);
[points] = sphere_points(input,radius);
fold_pnt=[];
for i=1:n

%According to the principal component or Total least squares, the plane fitting of the bounding sphere and the calculation of each normal vector are realized
    [parameter] = sphere_PCA(points{i}); % Plane fitting parameter
%Point cloud in the bounding ball projected onto the plane  
   [project_plane] = PC_Proj(parameter,points{i});
    
% three-dimensional point transformed into a two-dimensional point
    normal1=parameter(:,1:3);
    normal2=[0 0 1];
    [R]=Rotation_matrix(normal1,normal2);
    neighbor_pnt=project_plane'*R;
    x=neighbor_pnt(:,1);
    y=neighbor_pnt(:,2);
      
 %find the two farthest points 
    neighbor_number=size(neighbor_pnt,1);
    [neighbor_idx,dis]=knnsearch(neighbor_pnt,neighbor_pnt,'k', neighbor_number); 
    maxdis_idx=find(dis(:,neighbor_number)==max(dis(:,neighbor_number)));  
    farthest_idx=[maxdis_idx,neighbor_idx(maxdis_idx,neighbor_number)];
    farthest_idx= sort(farthest_idx');
    [farthest_idx,~,~]=unique( farthest_idx','rows');

    line_number=size(farthest_idx,1); %Make sure there are several pairs of farthest points
    
    %The distances from  point to line 

%     for j=1:1
    farthest_pnt=neighbor_pnt(farthest_idx(1,:),1:2); %Two farthest points coordinates
    B=[farthest_pnt(:,1),ones(2,1)];L=farthest_pnt(:,2);
    line_parameter=pinv(B'*B)*B'*L;%line constructed by two farthest points
    PL_dis=abs(line_parameter(1)*neighbor_pnt(:,1)-neighbor_pnt(:,2)+line_parameter(2))/sqrt(line_parameter(1)^2+1); %The distance between all the points and this line
%     end
    sort_dis=sort(PL_dis,'descend');%sort the distance 
     PL_std=std(PL_dis);
     
    %Calculate the number of points on the left and right sides of the line
    judge_side=line_parameter(1)*x+line_parameter(2)-y;
    right_side_number=length(find(judge_side>0));
    left_side_number=length(find(judge_side<0));
    if (right_side_number>DP_DS*left_side_number | left_side_number>DP_DS*right_side_number) & PL_std>PL_threshold 
        if length(PL_dis)<rank_dis_threshold+1
         [~,fold_line]=max(PL_dis);
        else
        fold_line=find(PL_dis>sort_dis(rank_dis_threshold+1,:));
        end
        fold_pnt=unique([fold_pnt;points{i}(fold_line,:)],'rows');
    end
end

