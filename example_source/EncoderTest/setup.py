from distutils.core import setup, Extension

setup(name="EncoderReader", version="1.0", description="EncoderReader"
	  ,author = "Jsharp", author_email="jsharp83@gmail.com", url = "http://isnl.kaist.ac.kr",
	  ext_modules = [Extension("EncoderReader", ["EncoderReader.c"])])
