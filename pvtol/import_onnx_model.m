% This file is used to save onnx file to a matlab function
modelfile = 'CDC_safe_explo.onnx';
% net = importONNXNetwork(modelfile,'OutputLayerType','regression')
params = importONNXFunction(modelfile,'CDC_safe_explo')

save('CDC_safe_explo.mat','params')