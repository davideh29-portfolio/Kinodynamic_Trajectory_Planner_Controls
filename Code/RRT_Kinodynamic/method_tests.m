%% Test Queue
clear;
% Initialize testing points
q_1 = .1 * ones(1, 5);
q_2 = .2 * ones(1, 5);
q_3 = .3 * ones(1, 5);
q_4 = .4 * ones(1, 5);
q_5 = .5 * ones(1, 5);
q_6 = .6 * ones(1, 5);
q_7 = .7 * ones(1, 5);
q_8 = .8 * ones(1, 5);
% Create nodes
n_1 = Node(q_1);
n_2 = Node(q_2);
n_3 = Node(q_3);
n_4 = Node(q_4);
n_5 = Node(q_5);
n_6 = Node(q_6);
n_7 = Node(q_7);
n_8 = Node(q_8);

% Initializing trees
n_1.children{1} = n_2;
n_2.parent{1} = n_1;
n_2.children{1} = n_3;
n_3.parent{1} = n_2;
n_1.children{2} = n_4;
n_4.parent{1} = n_1;

n_8.children{1} = n_7;
n_7.parent{1} = n_8;
n_8.children{2} = n_6;
n_6.parent{1} = n_8;
n_7.children{1} = n_5;
n_5.parent{1} = n_7;
n_5.children{1} = n_3;
n_3.parent{2} = n_5;

% Create queue w/recursive push
test_queue_1 = Queue();
test_queue_2 = Queue();
test_queue_1 = test_queue_1.push(n_1)
test_queue_2 = test_queue_2.push(n_8)

% Generate path from tree
path = genPath(n_1, n_8, n_3)
