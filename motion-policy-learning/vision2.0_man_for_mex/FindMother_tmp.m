function FindMother_tmp(j)
%% A one-shot function for setting up related joints


% for 'body' (or base)
if j == 1
    uLINK_tmp(j).mother = 0;
end

% use child to define its mother
% for non-ankle2 joints(including 'body'), which are not directly connected to feet
if uLINK_tmp(j).child ~= 0
    uLINK_tmp(uLINK_tmp(j).child).mother = j; % the mother of j's child is j
    % proceed to find 'mother's for each joint on the left leg(stops at Lankle2 whose 'child' is 0)
    FindMother(uLINK_tmp(j).child); 
end

% for 'Lhip1' only, for which .sister = 8; 8 is 'Rhip1'...
if uLINK_tmp(j).sister ~= 0
    uLINK_tmp(uLINK_tmp(j).sister).mother = uLINK_tmp(j).mother; % uLINK(8).mother = uLINK(2).mother
    % both uLINK(2).mother and uLINK(8).mother are undefined???
    
    % proceed to find 'mother's for each joint on the right leg(stops at Rankle2 whose 'child' is 0)
    FindMother(uLINK_tmp(j).sister); % FindMother(8)
end

end
