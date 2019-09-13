function [assignments,costs]= assign2DByGibbs(C,numIteration,k)
%%KBEST2DASSIGN Find the k lowest cost 2D assignments for the
%               two-dimensional assignment problem with a rectangular cost
%               matrix C.
%
%INPUTS: C A numRowXnumCol cost matrix. numRow = number of objects. numCol
%          = number of objects + number of measurements
%        numIteration: number of iterations used in Gibbs sampling
%        k The number >=1 of hypotheses to generate. If k is less than the
%          total number of unique hypotheses, then all possible hypotheses
%          will be returned.
%
%OUTPUTS: col4rowBest A numRowXk matrix where the entry in each element
%                     is an assignment of the element in that row to a
%                     column. 0 entries signify unassigned rows.
%            costs    A kX1 vector containing the sum of the values of the
%                     assigned elements in C for all of the hypotheses.

% Compute number of objects and measurements
n = size(C,1);
m = size(C,2) - n;

% Initialize assignments and costs with zeros
assignments = zeros(n,numIteration);
costs = zeros(numIteration,1);

% All missed detections as initial solution
currsoln= m+1:m+n; 
assignments(:,1)= currsoln;

% Compute initial cost
costs(1)=sum(C(sub2ind(size(C),1:n,currsoln)));

for sol= 2:numIteration %For each iteration
    for var = 1:n %For each object
        
        % Conversion of cost for a given object from NLL to likelihood 
        tempsamp = exp(-C(var,:)); 
        
        % Lock out current and previous iteration step assignments 
        % except for the one in question to get a feasible assignment
        tempsamp(currsoln([1:var-1,var+1:end])) = 0; 
        
        % Find likelihood costs that are greater than 0
        idxold= find(tempsamp>0); tempsamp = tempsamp(idxold);
        
        % Sample from the likelihood cost distribution
        [~,~,currsoln(var)]= histcounts(rand(1,1),[0;cumsum(tempsamp(:))/sum(tempsamp)]);
        
        % Update current solution
        currsoln(var)= idxold(currsoln(var));
    end
    
    % Coumpute Assignments and costs for the solution
    assignments(:,sol)= currsoln;
    costs(sol)= sum(C(sub2ind(size(C),1:n,currsoln)));
end

% Compute unique assignments and costs and store them
[unique_assignments,I,~] = unique(assignments','rows');
assignments = unique_assignments';
costs = costs(I);

% If the unique assignment size is bigger than the number of hypothesis to
% generate keep the k best solutions
if length(costs) > k
    [costs, sorted_idx] = sort(costs);
    costs = costs(1:k);
    assignments = assignments(:,sorted_idx(1:k));
end

end