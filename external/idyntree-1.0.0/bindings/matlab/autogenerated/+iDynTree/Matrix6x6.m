classdef Matrix6x6 < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = Matrix6x6(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(235, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = paren(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(236, self, varargin{:});
    end
    function varargout = getVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(237, self, varargin{:});
    end
    function varargout = setVal(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(238, self, varargin{:});
    end
    function varargout = rows(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(239, self, varargin{:});
    end
    function varargout = cols(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(240, self, varargin{:});
    end
    function varargout = data(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(241, self, varargin{:});
    end
    function varargout = zero(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(242, self, varargin{:});
    end
    function varargout = fillRowMajorBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(243, self, varargin{:});
    end
    function varargout = fillColMajorBuffer(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(244, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(245, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(246, self, varargin{:});
    end
    function varargout = toMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(247, self, varargin{:});
    end
    function varargout = fromMatlab(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(248, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(249, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
