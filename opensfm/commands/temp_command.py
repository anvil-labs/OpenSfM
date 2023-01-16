import json

# from opensfm.actions import create_rig

from . import command
import argparse
from opensfm.dataset import DataSet


class Command(command.CommandBase):
    name = "test_command"
    help = "TEMP"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        print('this is a success')

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
