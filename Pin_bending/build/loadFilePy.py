import os
import sys
import shutil
DEFAULT_FILE = 'automata.bin'
COMPILED_FILE = 'main.bin'

class LoadFile:
	# print "LoadFile class";
	def driveCheck(self):
		return os.path.isdir(DIR);
	
	def getDefaultPath(self):
		lst = [];
		lst.append(DIR);
		lst.append(DEFAULT_FILE);
		defPath = ''.join(lst);
		return defPath;

	def getCompiledPath(self):
		lst = [];
		lst.append(DIR);
		lst.append(COMPILED_FILE);
		defPath = ''.join(lst);
		return defPath;

	def deleteFile(self):
		if (os.path.exists(self.getDefaultPath())):
		# if (os.path.exists("G:/automata.bin")):
			os.remove(self.getDefaultPath())
		elif(os.path.exists(self.getCompiledPath())):
			os.remove(self.getCompiledPath())
		return 1;
	
	def copyBinFile(self):
		if(os.path.exists(COMPILED_FILE)):
			shutil.copy(COMPILED_FILE,DIR);
		return 1;

	def doEverything(self):
		if(self.driveCheck()):
			self.deleteFile();
			self.copyBinFile();
		else:
			print "Drive Not Connected!!!"
		return 1;

if __name__ == '__main__':
	DIR = sys.argv[1];
	loadFileObj = LoadFile();
	loadFileObj.doEverything();