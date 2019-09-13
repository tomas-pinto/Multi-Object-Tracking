function output = read_log(filename)

fileID = fopen(filename,'r');
formatSpec = '%d %f %f';
sizeA = [3 Inf];
A = fscanf(fileID,formatSpec,sizeA);

K = max(unique(A(1,:)))+1;
output = cell(K,1);

for k = 1:K
    output{k} = A(2:3,A(1,:) == k-1);
end