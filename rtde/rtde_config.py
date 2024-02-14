#  Copyright (c) 2024. By Benjamin Schmautz

import xml.etree.ElementTree as ET


class Recipe(object):
    __slots__ = ['key', 'names', 'types']

    @staticmethod
    def parse(recipe_node):
        rmd = Recipe()
        rmd.key = recipe_node.get('key')
        rmd.names = [f.get('name') for f in recipe_node.findall('field')]
        rmd.types = [f.get('type') for f in recipe_node.findall('field')]
        return rmd


class ConfigFile(object):
    def __init__(self, filename):
        self.__filename = filename
        tree = ET.parse(self.__filename)
        root = tree.getroot()
        recipes = [Recipe.parse(r) for r in root.findall('recipe')]
        self.__dictionary = dict()
        for r in recipes:
            self.__dictionary[r.key] = r

    def get_recipe(self, key):
        r = self.__dictionary[key]
        return r.names, r.types
