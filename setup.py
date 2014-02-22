from setuptools import setup

setup(name='motmot.fmf_saver',
      description='FMF saver plugin for FView',
      version='0.0.1',
      packages = find_packages(),
      packages = ['motmot', 'motmot.fmf_saver'],
      namespace_packages = ['motmot'],
      author='Andrew Straw',
      author_email='strawman@astraw.com',
      url='http://code.astraw.com/projects/motmot',
      entry_points = {
    'motmot.fview.plugins':'fmf_saver = motmot.fmf_saver.fmf_saver:FmfSaver',
    },
      )
