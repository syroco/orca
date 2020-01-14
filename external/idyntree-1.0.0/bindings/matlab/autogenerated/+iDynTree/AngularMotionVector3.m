classdef AngularMotionVector3 < iDynTree.MotionVector3__AngularMotionVector3
  methods
    function self = AngularMotionVector3(varargin)
      self@iDynTree.MotionVector3__AngularMotionVector3(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(508, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = exp(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(509, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(510, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
