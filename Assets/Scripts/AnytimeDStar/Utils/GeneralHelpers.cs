using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace ADStar
{
    public class GeneralHelpers
    {

        // Given two lists (of tuples) of the same size, check if the 1st list has the larger items.
        public static bool CheckIfList1ExceedsList2(List<double> list1, List<double> list2)
        {
            int checkLargetList = 0;
            for (int i = 0; i < list1.Count; i++)
            {
                if (list1[i] == list2[i]) { continue; }
                checkLargetList = list1[i] > list2[i] ? checkLargetList + 1 : checkLargetList - 1;
            }
            bool list1IsLarger = checkLargetList >= 0;
            return list1IsLarger;
        }

        // Check if list1 is smaller than list2.
        public static bool CheckIfList1IsSmallerThanList2(List<double> list1, List<double> list2)
        {
            int checkSmallestList = 0;
            for (int i = 0; i < list1.Count; i++)
            {
                if (list1[i] == list2[i]) { continue; }
                checkSmallestList = list1[i] < list2[i] ? checkSmallestList + 1 : checkSmallestList - 1;
            }
            bool list1IsSmaller = checkSmallestList > 0;
            return list1IsSmaller;
        }
        public static bool CheckIfList1IsSmallerThanList2_V2(List<double> list1, List<double> list2)
        {
            bool list1IsSmaller = false;
            double list1Sum = 0;
            double list2Sum = 0;
            for (int i = 0; i < list1.Count; i++)
            {
                list1Sum += list1[i];
                list2Sum += list2[i];
                if (i == list1.Count-1)
                {
                    if (list1Sum < list2Sum)
                    {
                        list1IsSmaller = true;
                    }
                    else
                    {
                        list1IsSmaller = false;
                    }
                }
            }
            return list1IsSmaller;
        }

        public static bool KeyCompare(List<double> list1, List<double> list2)
        {
            if (list1[0] + 0.000001 < list2[0])
                return true;
            else if (list1[0] - 0.000001 > list2[0])
                return false;
            else if (list1[1] + 0.000001 < list2[1])
                return true;
            else if (list1[1] - 0.000001 > list2[1])
                return false;

            return false;
        }

        public static string ReturnSetAsSingleString(IList setX)
        {
            string appendedString = "";
            foreach (var x in setX)
            {
                appendedString += x + " , ";
            }
            return appendedString;
        }
        public static string ReturnSetAsSingleString(HashSet<Tuple<float, float>> set)
        {
            string appendedString = "";
            foreach (var x in set)
            {
                appendedString += x + ", ";
            }
            return appendedString;
        }


        public static string ReturnListAsSingleString(IList list)
        {
            string appendedString = "";
            foreach (var x in list)
            {
                appendedString += x + ", ";
            }
            return appendedString;
        }

        public static string ReturnDictAsSingleString(IDictionary dict)
        {
            string appendedString = "";
            foreach (var x in dict.Keys)
            {
                appendedString += x + ", ";
            }
            return appendedString;
        }

        public static string ReturnSetAsSingleString(HashSet<Tuple<int, int>> set)
        {
            string appendedString = "";
            foreach (var x in set)
            {
                appendedString += x + ", ";
            }
            return appendedString;
        }

        public static string ReturnSplittedString(string str, char delimiter)
        {
            string appendedString = "";
            string[] splittedString = str.Split('-');
            foreach (var x in splittedString)
            {
                appendedString += x;
            }
            return appendedString;
        }

    }
}
