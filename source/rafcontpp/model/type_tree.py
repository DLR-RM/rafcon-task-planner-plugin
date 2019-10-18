# Copyright (C) 2018-2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>

# Don't connect with the Copyright comment above!
# Version 14.06.2019


class TypeTree:
    """ A class that helps to create a pddl-type-tree
    TypeTree is a datastructure that helps to create a pddl-type-tree out of
    a type databse containing type:parent entries.
    although it is a type tree, it has no advantages of a tree-datastructure.
    so its not sorted in a way, and is not fast in searching or inserting!
    """

    def __init__(self, type_name):
        """
        Needs the name of a type, and returns a type-tree-node. the node it self is a TypeTree.

        :param type_name: The name of a Type, can't be None!
        :return TypeTree: A TypeTree object
        """
        if type_name is None:
            raise ValueError('type_name must not be None!')
        self.type_name = type_name
        self.children = []

    def add_type_branch(self, type_name, type_dict):
        """
        addTypeBranch does not only add the given type, but also the whole branch.
        this means it also adds all parents, all children and their children
        to the type tree.

        :param type_name: The name of a type.
        :param type_dict: A type dictionary, containing the type.
        :return: Boolean: True if insert was successfull, false otherwise.
        """
        inserted = self.recursive_insert(type_name, type_dict)
        for_inserted = True
        if inserted:
            for key in type_dict.keys():
                if type_dict[key] == type_name:
                    for_inserted = for_inserted & self.add_type_branch(key, type_dict)
        return inserted and for_inserted

    def recursive_insert(self, type_name, type_dict):
        """
        recursiveInsert needs a typeName and a typeDict and inserts
        the type with it's parents into the tree.

        :param type_name: The name of a type.
        :param type_dict: A type dicitonary, containing the type.
        :return: Boolean: True if insert was successfull, false otherwise.
        """
        inserted = False
        if type_name in type_dict:
            parent = type_dict[type_name]
            if self.is_in_tree(parent):
                inserted = self.insert(type_name, parent)
            else:
                inserted = self.recursive_insert(parent, type_dict)
                if inserted:
                    inserted = self.insert(type_name, parent)
        return inserted or self.type_name == type_name

    def insert(self, type_name, parent_name):
        """
        insert takes a typeName and a parentName, and inserts it into the tree.
        insert does not work, if the parent is not in the tree yet.

        :param type_name: The name of the type to insert.
        :param parent_name: The direct parent of the type to insert.
        :return: Boolean: True if insert was successful, false otherwise. (for example if the parent is not in the tree yet.)
        """
        inserted = self.is_in_tree(type_name)
        if (not type_name is None) & (not inserted):
            inserted = self.__insert(type_name, parent_name)
        return inserted

    def __insert(self, type_name, parent_name):
        """
         __insert takes a typeName and a parentName, and inserts the typeName as child of the parentName into the tree.

         :param type_name: The name of the type to insert
         :param parent_name: The direct parent of the type to insert
         :return: Boolean: True if insert was successful, false otherwise. (for example if the parent is not in the tree yet.)
         """
        inserted = False
        if parent_name == self.type_name:
            self.children.append(TypeTree(type_name))
            inserted = True
        else:
            for child in self.children:
                inserted = child.__insert(type_name, parent_name)
                if inserted:
                    break
        return inserted

    def get_as_pddl_string(self):
        """
        Returns a string in pddl notation representing the type-tree.

        :return: String: Returns the typetree as pddl conform string.
        """
        as_string = ""
        if self.children:
            for child in self.children:
                as_string = as_string + child.type_name + " "
            as_string = as_string + "- " + self.type_name + "\r\n"
            for child in self.children:
                child_string = child.get_as_pddl_string()
                if child_string != child.type_name:
                    as_string = as_string + child_string
        else:
            as_string = self.type_name
        return as_string

    def get_as_list(self):
        """
        Takes the Tree, and writes all its elements into a list

        :return: [String]: A list, contining all types of the tree
        """
        type_list = [self.type_name]
        for child in self.children:
            type_list.extend(child.get_as_list())
        return type_list

    def is_in_tree(self, type_to_search):
        """
        isInTree searchs the typeTree for a specific type, and returns true,
        if the tree contains the type.
        unfortunately the tree is not sorted, and has a complexity of O(n)

        :param type_name: The name of the type to search.
        :return: Boolean: True if the tree contains the type, else false.
        """
        return self.get_sub_tree(type_to_search) is not None

    def get_sub_tree(self, type_to_search):
        """ get_sub_tree
        get_sub_tree gets a type, and returns the subtree, with the type as root.

        :param type_to_search: The root of the subtree to get
        :return: TypeTree: A sub tree, or None if the type is not in the tree.
        """
        sub_tree = None
        if self.type_name == type_to_search:
            sub_tree = self
        else:
            for child in self.children:
                sub_tree = child.get_sub_tree(type_to_search)

                if sub_tree is not None:
                    break
        return sub_tree

    def is_parent_of(self, parent, child):
        """ is_parent_of
        is_parent_of receives two types, and returns true,
        if the first type it the parent of the second type.

        :param parent: The maybe parent type.
        :param child: The maybe child type.
        :return: Boolean: True, if the parent is really the parent of the child, false otherwise.
        """
        is_parent = False
        if parent != child:
            sub_tree = self.get_sub_tree(parent)
            if sub_tree is not None:
                is_parent = sub_tree.is_in_tree(child)
        return is_parent

    def get_parent_of(self, type_name):
        """
        get_parent_of receives a type name, and searchs in the tree for it's parent.

        :param type_name: A type name
        :return: String: The parent name or None if the type has no parent or is not in tree.
        """
        parent = None
        # look, if this node the parent
        for child in self.children:
            if type_name == child.type_name:
                parent = self.type_name
                break

        # search in child nodes for parent
        if parent is None:
            for child in self.children:
                parent = child.get_parent_of(type_name)
                if parent:
                    break
        return parent

    def get_smallest_parent(self, type_a, type_b):
        """
        gets two types and finds their smallest parent. E.g. given a type t1 and a type t2:
        if: t2 is parent of t1, it will return t2
        if t1 is the parent of t2, it will return t1
        if t3 is the parent of t1 and t2, it will return t3
        if t1 or t2 is not in the tree, it will return None.

        :param type_a: A type a
        :param type_b: A type b
        :return: String: The smallest parent, or None
        """
        smallest_parent = None
        # fast fail if not in tree, or one is none
        if type_a is None or type_b is None:
            return None
        if not self.is_in_tree(type_a) or not self.is_in_tree(type_b):
            return None
        # check if one is the parent of the other
        smallest_parent = type_a if self.is_parent_of(type_a, type_b) else smallest_parent
        smallest_parent = type_b if self.is_parent_of(type_b, type_a) else smallest_parent
        # climb up branches until branch root
        if smallest_parent is None:
            parent_a = self.get_parent_of(type_a)
            parent_b = self.get_parent_of(type_b)
            if parent_a and parent_b:
                while parent_a and smallest_parent is None:
                    c_parent_b = parent_b
                    while c_parent_b and smallest_parent is None:
                        if c_parent_b == parent_a:
                            smallest_parent = c_parent_b
                        else:
                            c_parent_b = self.get_parent_of(c_parent_b)
                    parent_a = self.get_parent_of(parent_a)
        return smallest_parent
