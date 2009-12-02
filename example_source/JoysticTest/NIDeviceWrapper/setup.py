from distutils.core import setup, Extension

setup(name="NIDeviceWrapper", version="1.0", description="NI Driver Connector"
	  ,author = "Jsharp", author_email="jsharp83@gmail.com", url = "http://isnl.kaist.ac.kr",
	  ext_modules = [Extension("NIDeviceWrapper", ["NIDeviceWrapper.c"])])
