function x = mysoftplus(x)
%RELU   Apply rectified linear unit activation
%
%   Y = RELU(X) computes the relu activation for the input data X. X can be
%   a formatted or an unformatted dlarray. If X is a formatted dlarray,
%   output Y is a formatted dlarray with the same dimension labels as X. If
%   X is an unformatted dlarray, output Y is an unformatted dlarray.
%
%   The relu operation performs a threshold operation, where any input
%   value less than zero is set to zero. This is equivalent to:
%       Y = X;       % If X > 0
%       Y = 0;       % If X <= 0
%
%   Example:
%       % Create input data as a formatted dlarray and compute relu
%       % activation
%       x = dlarray(randn(8,8,3,16),'SSCB');
%       y = relu(x);
%
%   See also BATCHNORM, DLARRAY, LEAKYRELU

%   Copyright 2019 The MathWorks, Inc.

% Extract the input data
% fd = x.FormattedData;
% x.FormattedData = [];
% [fd,xdata] = extractData(fd);
xdata = x.extractdata;

% The dlarray methods should not accept logical x 
if islogical(xdata)
    error(message('deep:dlarray:LogicalsNotSupported'));
end

% Call the internal API
xdata = softplus(xdata);

% Format is guaranteed not to have changed
% fd = insertData(fd, xdata);
% x.FormattedData = fd;
x = dlarray(xdata);
end