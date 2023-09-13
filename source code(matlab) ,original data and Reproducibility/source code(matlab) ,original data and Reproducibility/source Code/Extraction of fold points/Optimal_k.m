
%�������ǻ�����Ϣ������ʵ�ֵ�����ڽ������ж�׼��
%�ο����ף�¬ά��,���״�,������,��. �󳡾��ڽ����������ȡ��ƽ��ָ��㷨[J]. �й�����,2015,42(9):336-342. 
%�������ݣ�ԭʼ����input_pnts��nx3����[min_k max_k] ���ڽ������ķ�Χ
%������ݣ�ÿ���㲻ͬ�ڽ������Ϣ��E��ÿ���������ڽ�����best_k
function [E,best_k] = Optimal_k(input_pnts,min_k,max_k)


inter=max_k-min_k+1;
[neigh,~] = knnsearch(input_pnts,input_pnts, 'k', max_k);
for i=1:size(input_pnts,1)
for j=1:inter
    k=min_k+(j-1);
pnt=input_pnts(neigh(i,1:k),:);
n=size(pnt,1);
mean_pnts=mean(pnt,1);
M=pnt-repmat(mean_pnts,n,1);
[U,S,V] = svd(M'*M); %U ��V��������������S������ֵ

a1=S(1,1);a2=S(2,2);a3=S(3,3);

a11=(sqrt(a1)-sqrt(a2))/sqrt(a1);
a12=(sqrt(a2)-sqrt(a3))/sqrt(a1);
a13=(sqrt(a3))/sqrt(a1);
E(j,i)=-a11*log(a11)-a12*log(a12)-a13*log(a13);
end
end
[min_E,index]=min(E,[],1);
best_k=min_k+(index-1);
