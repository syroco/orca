function v = POINT_LIGHT()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 50);
  end
  v = vInitialized;
end
