function [output, state] = uncer_func_better(input, params, varargin)
%UNCER_FUNC_BETTER Function implementing an imported ONNX network.
%
% THIS FILE WAS AUTO-GENERATED BY importONNXFunction.
% ONNX Operator Set Version: 11
%
% Variable names in this function are taken from the original ONNX file.
%
% [OUTPUT] = uncer_func_better(INPUT, PARAMS)
%			- Evaluates the imported ONNX network UNCER_FUNC_BETTER with input(s)
%			INPUT and the imported network parameters in PARAMS. Returns
%			network output(s) in OUTPUT.
%
% [OUTPUT, STATE] = uncer_func_better(INPUT, PARAMS)
%			- Additionally returns state variables in STATE. When training,
%			use this form and set TRAINING to true.
%
% [__] = uncer_func_better(INPUT, PARAMS, 'NAME1', VAL1, 'NAME2', VAL2, ...)
%			- Specifies additional name-value pairs described below:
%
% 'Training'
% 			Boolean indicating whether the network is being evaluated for
%			prediction or training. If TRAINING is true, state variables
%			will be updated.
%
% 'InputDataPermutation'
%			'auto' - Automatically attempt to determine the permutation
%			 between the dimensions of the input data and the dimensions of
%			the ONNX model input. For example, the permutation from HWCN
%			(MATLAB standard) to NCHW (ONNX standard) uses the vector
%			[4 3 1 2]. See the documentation for IMPORTONNXFUNCTION for
%			more information about automatic permutation.
%
%			'none' - Input(s) are passed in the ONNX model format. See 'Inputs'.
%
%			numeric vector - The permutation vector describing the
%			transformation between input data dimensions and the expected
%			ONNX input dimensions.%
%			cell array - If the network has multiple inputs, each cell
%			contains 'auto', 'none', or a numeric vector.
%
% 'OutputDataPermutation'
%			'auto' - Automatically attempt to determine the permutation
%			between the dimensions of the output and a conventional MATLAB
%			dimension ordering. For example, the permutation from NC (ONNX
%			standard) to CN (MATLAB standard) uses the vector [2 1]. See
%			the documentation for IMPORTONNXFUNCTION for more information
%			about automatic permutation.
%
%			'none' - Return output(s) as given by the ONNX model. See 'Outputs'.
%
%			numeric vector - The permutation vector describing the
%			transformation between the ONNX output dimensions and the
%			desired output dimensions.%
%			cell array - If the network has multiple outputs, each cell
%			contains 'auto', 'none' or a numeric vector.
%
% Inputs:
% -------
% INPUT
%			- Input(s) to the ONNX network.
%			  The input size(s) expected by the ONNX file are:
%				  INPUT:		[batch_size, 4]				Type: FLOAT
%			  By default, the function will try to permute the input(s)
%			  into this dimension ordering. If the default is incorrect,
%			  use the 'InputDataPermutation' argument to control the
%			  permutation.
%
%
% PARAMS	- Network parameters returned by 'importONNXFunction'.
%
%
% Outputs:
% --------
% OUTPUT
%			- Output(s) of the ONNX network.
%			  Without permutation, the size(s) of the outputs are:
%				  OUTPUT:		[batch_size, 2]				Type: FLOAT
%			  By default, the function will try to permute the output(s)
%			  from this dimension ordering into a conventional MATLAB
%			  ordering. If the default is incorrect, use the
%			  'OutputDataPermutation' argument to control the permutation.
%
% STATE		- (Optional) State variables. When TRAINING is true, these will
% 			  have been updated from the original values in PARAMS.State.
%
%
%  See also importONNXFunction

% Preprocess the input data and arguments:
[input, Training, outputDataPerms, anyDlarrayInputs] = preprocessInput(input, params, varargin{:});
% Put all variables into a single struct to implement dynamic scoping:
[Vars, NumDims] = packageVariables(params, {'input'}, {input}, [2]);
% Call the top-level graph function:
[output, outputNumDims, state] = torch_jit_exportGraph1000(input, NumDims.input, Vars, NumDims, Training, params.State);
% Postprocess the output data
[output] = postprocessOutput(output, outputDataPerms, anyDlarrayInputs, Training, varargin{:});
end

function [output, outputNumDims1012, state] = torch_jit_exportGraph1000(input, inputNumDims1011, Vars, NumDims, Training, state)
% Function implementing the graph 'torch_jit_exportGraph1000'
% Update Vars and NumDims from the graph's formal input parameters. Note that state variables are already in Vars.
Vars.input = input;
NumDims.input = inputNumDims1011;

% Execute the operators:
% Flatten:
[dim1, dim2, NumDims.x21] = prepareFlattenArgs(Vars.w1_parametrizations_weight_original, 1, NumDims.w1_parametrizations_weight_original);
Vars.x21 = reshape(Vars.w1_parametrizations_weight_original, dim1, dim2);

% MatMul:
[Vars.x22, NumDims.x22] = onnxMatMul(Vars.x21, Vars.w1_parametrizations_weight_0__v, NumDims.x21, NumDims.w1_parametrizations_weight_0__v);

% MatMul:
[Vars.x23, NumDims.x23] = onnxMatMul(Vars.w1_parametrizations_weight_0__u, Vars.x22, NumDims.w1_parametrizations_weight_0__u, NumDims.x22);

% Div:
Vars.x24 = Vars.x21 ./ Vars.x23;
NumDims.x24 = max(NumDims.x21, NumDims.x23);

% Gemm:
[A, B, C, alpha, beta, NumDims.x25] = prepareGemmArgs(Vars.input, Vars.x24, Vars.w1_bias, Vars.Gemmalpha1001, Vars.Gemmbeta1002, 0, 1, NumDims.w1_bias);
Vars.x25 = alpha*B*A + beta*C;

% Mul:
Vars.x27 = Vars.x25 .* Vars.x26;
NumDims.x27 = max(NumDims.x25, NumDims.x26);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Softplus):
Vars.x28 = mysoftplus(Vars.x27);
NumDims.x28 = NumDims.x27;

% Flatten:
[dim1, dim2, NumDims.x29] = prepareFlattenArgs(Vars.w2_parametrizations_weight_original, 1, NumDims.w2_parametrizations_weight_original);
Vars.x29 = reshape(Vars.w2_parametrizations_weight_original, dim1, dim2);

% MatMul:
[Vars.x30, NumDims.x30] = onnxMatMul(Vars.x29, Vars.w2_parametrizations_weight_0__v, NumDims.x29, NumDims.w2_parametrizations_weight_0__v);

% MatMul:
[Vars.x31, NumDims.x31] = onnxMatMul(Vars.w2_parametrizations_weight_0__u, Vars.x30, NumDims.w2_parametrizations_weight_0__u, NumDims.x30);

% Div:
Vars.x32 = Vars.x29 ./ Vars.x31;
NumDims.x32 = max(NumDims.x29, NumDims.x31);

% Gemm:
[A, B, C, alpha, beta, NumDims.x33] = prepareGemmArgs(Vars.x28, Vars.x32, Vars.w2_bias, Vars.Gemmalpha1003, Vars.Gemmbeta1004, 0, 1, NumDims.w2_bias);
Vars.x33 = alpha*B*A + beta*C;

% Mul:
Vars.x35 = Vars.x33 .* Vars.x34;
NumDims.x35 = max(NumDims.x33, NumDims.x34);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Softplus):
Vars.x36 = mysoftplus(Vars.x35);
NumDims.x36 = NumDims.x35;

% Flatten:
[dim1, dim2, NumDims.x37] = prepareFlattenArgs(Vars.w3_parametrizations_weight_original, 1, NumDims.w3_parametrizations_weight_original);
Vars.x37 = reshape(Vars.w3_parametrizations_weight_original, dim1, dim2);

% MatMul:
[Vars.x38, NumDims.x38] = onnxMatMul(Vars.x37, Vars.w3_parametrizations_weight_0__v, NumDims.x37, NumDims.w3_parametrizations_weight_0__v);

% MatMul:
[Vars.x39, NumDims.x39] = onnxMatMul(Vars.w3_parametrizations_weight_0__u, Vars.x38, NumDims.w3_parametrizations_weight_0__u, NumDims.x38);

% Div:
Vars.x40 = Vars.x37 ./ Vars.x39;
NumDims.x40 = max(NumDims.x37, NumDims.x39);

% Gemm:
[A, B, C, alpha, beta, NumDims.x41] = prepareGemmArgs(Vars.x36, Vars.x40, Vars.w3_bias, Vars.Gemmalpha1005, Vars.Gemmbeta1006, 0, 1, NumDims.w3_bias);
Vars.x41 = alpha*B*A + beta*C;

% Mul:
Vars.x43 = Vars.x41 .* Vars.x42;
NumDims.x43 = max(NumDims.x41, NumDims.x42);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Softplus):
Vars.x44 = mysoftplus(Vars.x43);
NumDims.x44 = NumDims.x43;

% Flatten:
[dim1, dim2, NumDims.x45] = prepareFlattenArgs(Vars.w4_parametrizations_weight_original, 1, NumDims.w4_parametrizations_weight_original);
Vars.x45 = reshape(Vars.w4_parametrizations_weight_original, dim1, dim2);

% MatMul:
[Vars.x46, NumDims.x46] = onnxMatMul(Vars.x45, Vars.w4_parametrizations_weight_0__v, NumDims.x45, NumDims.w4_parametrizations_weight_0__v);

% MatMul:
[Vars.x47, NumDims.x47] = onnxMatMul(Vars.w4_parametrizations_weight_0__u, Vars.x46, NumDims.w4_parametrizations_weight_0__u, NumDims.x46);

% Div:
Vars.x48 = Vars.x45 ./ Vars.x47;
NumDims.x48 = max(NumDims.x45, NumDims.x47);

% Gemm:
[A, B, C, alpha, beta, NumDims.x49] = prepareGemmArgs(Vars.x44, Vars.x48, Vars.w4_bias, Vars.Gemmalpha1007, Vars.Gemmbeta1008, 0, 1, NumDims.w4_bias);
Vars.x49 = alpha*B*A + beta*C;

% Mul:
Vars.x51 = Vars.x49 .* Vars.x50;
NumDims.x51 = max(NumDims.x49, NumDims.x50);

% PLACEHOLDER FUNCTION FOR UNSUPPORTED OPERATOR (Softplus):
Vars.x52 = mysoftplus(Vars.x51);
NumDims.x52 = NumDims.x51;

% Flatten:
[dim1, dim2, NumDims.x53] = prepareFlattenArgs(Vars.w5_parametrizations_weight_original, 1, NumDims.w5_parametrizations_weight_original);
Vars.x53 = reshape(Vars.w5_parametrizations_weight_original, dim1, dim2);

% MatMul:
[Vars.x54, NumDims.x54] = onnxMatMul(Vars.x53, Vars.w5_parametrizations_weight_0__v, NumDims.x53, NumDims.w5_parametrizations_weight_0__v);

% MatMul:
[Vars.x55, NumDims.x55] = onnxMatMul(Vars.w5_parametrizations_weight_0__u, Vars.x54, NumDims.w5_parametrizations_weight_0__u, NumDims.x54);

% Div:
Vars.x56 = Vars.x53 ./ Vars.x55;
NumDims.x56 = max(NumDims.x53, NumDims.x55);

% Gemm:
[A, B, C, alpha, beta, NumDims.x57] = prepareGemmArgs(Vars.x52, Vars.x56, Vars.w5_bias, Vars.Gemmalpha1009, Vars.Gemmbeta1010, 0, 1, NumDims.w5_bias);
Vars.x57 = alpha*B*A + beta*C;

% Mul:
Vars.output = Vars.x57 .* Vars.x58;
NumDims.output = max(NumDims.x57, NumDims.x58);

% Set graph output arguments from Vars and NumDims:
output = Vars.output;
outputNumDims1012 = NumDims.output;
% Set output state from Vars:
state = updateStruct(state, Vars);
end

function [inputDataPerms, outputDataPerms, Training] = parseInputs(input, numDataOutputs, params, varargin)
% Function to validate inputs to uncer_func_better:
p = inputParser;
isValidArrayInput = @(x)isnumeric(x) || isstring(x);
isValidONNXParameters = @(x)isa(x, 'ONNXParameters');
addRequired(p, 'input', isValidArrayInput);
addRequired(p, 'params', isValidONNXParameters);
addParameter(p, 'InputDataPermutation', 'auto');
addParameter(p, 'OutputDataPermutation', 'auto');
addParameter(p, 'Training', false);
parse(p, input, params, varargin{:});
inputDataPerms = p.Results.InputDataPermutation;
outputDataPerms = p.Results.OutputDataPermutation;
Training = p.Results.Training;
if isnumeric(inputDataPerms)
    inputDataPerms = {inputDataPerms};
end
if isstring(inputDataPerms) && isscalar(inputDataPerms) || ischar(inputDataPerms)
    inputDataPerms = repmat({inputDataPerms},1,1);
end
if isnumeric(outputDataPerms)
    outputDataPerms = {outputDataPerms};
end
if isstring(outputDataPerms) && isscalar(outputDataPerms) || ischar(outputDataPerms)
    outputDataPerms = repmat({outputDataPerms},1,numDataOutputs);
end
end

function [input, Training, outputDataPerms, anyDlarrayInputs] = preprocessInput(input, params, varargin)
% Parse input arguments
[inputDataPerms, outputDataPerms, Training] = parseInputs(input, 1, params, varargin{:});
anyDlarrayInputs = any(cellfun(@(x)isa(x, 'dlarray'), {input}));
% Make the input variables into unlabelled dlarrays:
input = makeUnlabeledDlarray(input);
% Permute inputs if requested:
input = permuteInputVar(input, inputDataPerms{1}, 2);
% Check input size(s):
checkInputSize(size(input), {'batch_size' 4}, "input");
end

function [output] = postprocessOutput(output, outputDataPerms, anyDlarrayInputs, Training, varargin)
% Set output type:
if ~anyDlarrayInputs && ~Training
    if isdlarray(output)
        output = extractdata(output);
    end
end
% Permute outputs if requested:
output = permuteOutputVar(output, outputDataPerms{1}, 2);
end


%% dlarray functions implementing ONNX operators:

function [D, numDimsD] = onnxMatMul(A, B, numDimsA, numDimsB)
% Implements the ONNX MatMul operator.

% If either arg is more than 2D, loop over all dimensions before the final
% 2. Inside the loop, perform matrix multiplication.

% If B is 1-D, temporarily extend it to a row vector
if numDimsB==1
    B = B(:)';
end
maxNumDims = max(numDimsA, numDimsB);
numDimsD = maxNumDims;
if maxNumDims > 2
    % sizes of matrices to be multiplied
    matSizeA        = size(A, 1:2);
    matSizeB        = size(B, 1:2);
    % size of the stack of matrices
    stackSizeA      = size(A, 3:maxNumDims);
    stackSizeB      = size(B, 3:maxNumDims);
    % final stack size
    resultStackSize = max(stackSizeA, stackSizeB);
    % full implicitly-expanded sizes
    fullSizeA       = [matSizeA resultStackSize];
    fullSizeB       = [matSizeB resultStackSize];
    resultSize      = [matSizeB(1) matSizeA(2) resultStackSize];
    % Repmat A and B up to the full stack size using implicit expansion
    A = A + zeros(fullSizeA);
    B = B + zeros(fullSizeB);
    % Reshape A and B to flatten the stack dims (all dims after the first 2)
    A2 = reshape(A, size(A,1), size(A,2), []);
    B2 = reshape(B, size(B,1), size(B,2), []);
    % Iterate down the stack dim, doing the 2d matrix multiplications
    D2 = zeros([matSizeB(1), matSizeA(2), size(A2,3)], 'like', A);
    for i = size(A2,3):-1:1
        D2(:,:,i) = B2(:,:,i) * A2(:,:,i);
    end
    % Reshape D2 to the result size (unflatten the stack dims)
    D = reshape(D2, resultSize);
else
    D = B * A;
    if numDimsA==1 || numDimsB==1
        D = D(:);
        numDimsD = 1;
    end
end
end

function [dim1, dim2, numDimsY] = prepareFlattenArgs(X, ONNXAxis, numDimsX)
% Prepares arguments for implementing the ONNX Flatten operator

% ONNXAxis is the number of dimensions that go on the left in ONNX, so here
% it is the number of dimensions that go on the right.
if ONNXAxis < 0
    ONNXAxis = ONNXAxis + numDimsX;
end
if ONNXAxis == 0
    dim2 = 1;
else
    dim2 = prod(size(X, numDimsX+1-ONNXAxis:numDimsX));     % numel on the right
end
dim1 = numel(X)/dim2;                                   % numel on the left
numDimsY = 2;
end

function [A, B, C, alpha, beta, numDimsY] = prepareGemmArgs(A, B, C, alpha, beta, transA, transB, numDimsC)
% Prepares arguments for implementing the ONNX Gemm operator
if transA
    A = A';
end
if transB
    B = B';
end
if numDimsC < 2
    C = C(:);   % C can be broadcast to [N M]. Make C a col vector ([N 1])
end
numDimsY = 2;
% Y=B*A because we want (AB)'=B'A', and B and A are already transposed.
end

%% Utility functions:

function s = appendStructs(varargin)
% s = appendStructs(s1, s2,...). Assign all fields in s1, s2,... into s.
if isempty(varargin)
    s = struct;
else
    s = varargin{1};
    for i = 2:numel(varargin)
        fromstr = varargin{i};
        fs = fieldnames(fromstr);
        for j = 1:numel(fs)
            s.(fs{j}) = fromstr.(fs{j});
        end
    end
end
end

function checkInputSize(inputShape, expectedShape, inputName)

if numel(expectedShape)==0
    % The input is a scalar
    if ~isequal(inputShape, [1 1])
        inputSizeStr = makeSizeString(inputShape);
        error(message('nnet_cnn_onnx:onnx:InputNeedsResize',inputName, "[1,1]", inputSizeStr));
    end
elseif numel(expectedShape)==1
    % The input is a vector
    if ~shapeIsColumnVector(inputShape) || ~iSizesMatch({inputShape(1)}, expectedShape)
        expectedShape{2} = 1;
        expectedSizeStr = makeSizeString(expectedShape);
        inputSizeStr = makeSizeString(inputShape);
        error(message('nnet_cnn_onnx:onnx:InputNeedsResize',inputName, expectedSizeStr, inputSizeStr));
    end
else
    % The input has 2 dimensions or more
    
    % The input dimensions have been reversed; flip them back to compare to the
    % expected ONNX shape.
    inputShape = fliplr(inputShape);
    
    % If the expected shape has fewer dims than the input shape, error.
    if numel(expectedShape) < numel(inputShape)
        expectedSizeStr = strjoin(["[", strjoin(string(expectedShape), ","), "]"], "");
        error(message('nnet_cnn_onnx:onnx:InputHasGreaterNDims', inputName, expectedSizeStr));
    end
    
    % Prepad the input shape with trailing ones up to the number of elements in
    % expectedShape
    inputShape = num2cell([ones(1, numel(expectedShape) - length(inputShape)) inputShape]);
    
    % Find the number of variable size dimensions in the expected shape
    numVariableInputs = sum(cellfun(@(x) isa(x, 'char') || isa(x, 'string'), expectedShape));
    
    % Find the number of input dimensions that are not in the expected shape
    % and cannot be represented by a variable dimension
    nonMatchingInputDims = setdiff(string(inputShape), string(expectedShape));
    numNonMatchingInputDims  = numel(nonMatchingInputDims) - numVariableInputs;
    
    expectedSizeStr = makeSizeString(expectedShape);
    inputSizeStr = makeSizeString(inputShape);
    if numNonMatchingInputDims == 0 && ~iSizesMatch(inputShape, expectedShape)
        % The actual and expected input dimensions match, but in
        % a different order. The input needs to be permuted.
        error(message('nnet_cnn_onnx:onnx:InputNeedsPermute',inputName, expectedSizeStr, inputSizeStr));
    elseif numNonMatchingInputDims > 0
        % The actual and expected input sizes do not match.
        error(message('nnet_cnn_onnx:onnx:InputNeedsResize',inputName, expectedSizeStr, inputSizeStr));
    end
end
end

function doesMatch = iSizesMatch(inputShape, expectedShape)
% Check whether the input and expected shapes match, in order.
% Size elements match if (1) the elements are equal, or (2) the expected
% size element is a variable (represented by a character vector or string)
doesMatch = true;
for i=1:numel(inputShape)
    if ~(isequal(inputShape{i},expectedShape{i}) || ischar(expectedShape{i}) || isstring(expectedShape{i}))
        doesMatch = false;
        return
    end
end
end

function sizeStr = makeSizeString(shape)
sizeStr = strjoin(["[", strjoin(string(shape), ","), "]"], "");
end

function isVec = shapeIsColumnVector(shape)
if numel(shape) == 2 && shape(2) == 1
    isVec = true;
else
    isVec = false;
end
end
function X = makeUnlabeledDlarray(X)
% Make numeric X into an unlabelled dlarray
if isa(X, 'dlarray')
    X = stripdims(X);
elseif isnumeric(X)
    if isinteger(X)
        % Make ints double so they can combine with anything without
        % reducing precision
        X = double(X);
    end
    X = dlarray(X);
end
end

function [Vars, NumDims] = packageVariables(params, inputNames, inputValues, inputNumDims)
% inputNames, inputValues are cell arrays. inputRanks is a numeric vector.
Vars = appendStructs(params.Learnables, params.Nonlearnables, params.State);
NumDims = params.NumDimensions;
% Add graph inputs
for i = 1:numel(inputNames)
    Vars.(inputNames{i}) = inputValues{i};
    NumDims.(inputNames{i}) = inputNumDims(i);
end
end

function X = permuteInputVar(X, userDataPerm, onnxNDims)
% Returns reverse-ONNX ordering
if onnxNDims == 0
    return;
elseif onnxNDims == 1 && isvector(X)
    X = X(:);
    return;
elseif isnumeric(userDataPerm)
    % Permute into reverse ONNX ordering
    if numel(userDataPerm) ~= onnxNDims
        error(message('nnet_cnn_onnx:onnx:InputPermutationSize', numel(userDataPerm), onnxNDims));
    end
    perm = fliplr(userDataPerm);
elseif isequal(userDataPerm, 'auto') && onnxNDims == 4
    % Permute MATLAB HWCN to reverse onnx (WHCN)
    perm = [2 1 3 4];
elseif isequal(userDataPerm, 'as-is')
    % Do not permute the input
    perm = 1:ndims(X);
else
    % userDataPerm is either 'none' or 'auto' with no default, which means
    % it's already in onnx ordering, so just make it reverse onnx
    perm = max(2,onnxNDims):-1:1;
end
X = permute(X, perm);
end

function Y = permuteOutputVar(Y, userDataPerm, onnxNDims)
switch onnxNDims
    case 0
        perm = [];
    case 1
        if isnumeric(userDataPerm)
            % Use the user's permutation because Y is a column vector which
            % already matches ONNX.
            perm = userDataPerm;
        elseif isequal(userDataPerm, 'auto')
            % Treat the 1D onnx vector as a 2D column and transpose it
            perm = [2 1];
        else
            % userDataPerm is 'none'. Leave Y alone because it already
            % matches onnx.
            perm = [];
        end
    otherwise
        % ndims >= 2
        if isnumeric(userDataPerm)
            % Use the inverse of the user's permutation. This is not just the
            % flip of the permutation vector.
            perm = onnxNDims + 1 - userDataPerm;
        elseif isequal(userDataPerm, 'auto')
            if onnxNDims == 2
                % Permute reverse ONNX CN to DLT CN (do nothing)
                perm = [];
            elseif onnxNDims == 4
                % Permute reverse onnx (WHCN) to MATLAB HWCN
                perm = [2 1 3 4];
            else
                % User wants the output in ONNX ordering, so just reverse it from
                % reverse onnx
                perm = onnxNDims:-1:1;
            end
        elseif isequal(userDataPerm, 'as-is')
            % Do not permute the input
            perm = 1:ndims(Y);
        else
            % userDataPerm is 'none', so just make it reverse onnx
            perm = onnxNDims:-1:1;
        end
end
if ~isempty(perm)
    Y = permute(Y, perm);
end
end

function s = updateStruct(s, t)
% Set all existing fields in s from fields in t, ignoring extra fields in t.
for name = transpose(fieldnames(s))
    s.(name{1}) = t.(name{1});
end
end
