classdef BerdyDynamicVariable < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function varargout = type(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1604, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1605, self, varargin{1});
      end
    end
    function varargout = id(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1606, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1607, self, varargin{1});
      end
    end
    function varargout = range(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1608, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1609, self, varargin{1});
      end
    end
    function varargout = eq(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1610, self, varargin{:});
    end
    function varargout = lt(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1611, self, varargin{:});
    end
    function self = BerdyDynamicVariable(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1612, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1613, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
