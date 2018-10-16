/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the QtXmlPatterns module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL21$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see http://www.qt.io/terms-conditions. For further
** information use the contact form at http://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 or version 3 as published by the Free
** Software Foundation and appearing in the file LICENSE.LGPLv21 and
** LICENSE.LGPLv3 included in the packaging of this file. Please review the
** following information to ensure the GNU Lesser General Public License
** requirements will be met: https://www.gnu.org/licenses/lgpl.html and
** http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** As a special exception, The Qt Company gives you certain additional
** rights. These rights are described in The Qt Company LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** $QT_END_LICENSE$
**
****************************************************************************/

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.

/* A Bison parser, made by GNU Bison 3.0.2.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2013 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

#ifndef YY_XPATH_QQUERYTRANSFORMPARSER_P_H_INCLUDED
# define YY_XPATH_QQUERYTRANSFORMPARSER_P_H_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int XPathdebug;
#endif

/* Token type.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    T_END_OF_FILE = 0,
    T_STRING_LITERAL = 258,
    T_NON_BOUNDARY_WS = 259,
    T_XPATH2_STRING_LITERAL = 260,
    T_QNAME = 261,
    T_NCNAME = 262,
    T_CLARK_NAME = 263,
    T_ANY_LOCAL_NAME = 264,
    T_ANY_PREFIX = 265,
    T_NUMBER = 266,
    T_XPATH2_NUMBER = 267,
    T_ANCESTOR = 268,
    T_ANCESTOR_OR_SELF = 269,
    T_AND = 270,
    T_APOS = 271,
    T_APPLY_TEMPLATE = 272,
    T_AS = 273,
    T_ASCENDING = 274,
    T_ASSIGN = 275,
    T_AT = 276,
    T_AT_SIGN = 277,
    T_ATTRIBUTE = 278,
    T_AVT = 279,
    T_BAR = 280,
    T_BASEURI = 281,
    T_BEGIN_END_TAG = 282,
    T_BOUNDARY_SPACE = 283,
    T_BY = 284,
    T_CALL_TEMPLATE = 285,
    T_CASE = 286,
    T_CASTABLE = 287,
    T_CAST = 288,
    T_CHILD = 289,
    T_COLLATION = 290,
    T_COLONCOLON = 291,
    T_COMMA = 292,
    T_COMMENT = 293,
    T_COMMENT_START = 294,
    T_CONSTRUCTION = 295,
    T_COPY_NAMESPACES = 296,
    T_CURLY_LBRACE = 297,
    T_CURLY_RBRACE = 298,
    T_DECLARE = 299,
    T_DEFAULT = 300,
    T_DESCENDANT = 301,
    T_DESCENDANT_OR_SELF = 302,
    T_DESCENDING = 303,
    T_DIV = 304,
    T_DOCUMENT = 305,
    T_DOCUMENT_NODE = 306,
    T_DOLLAR = 307,
    T_DOT = 308,
    T_DOTDOT = 309,
    T_ELEMENT = 310,
    T_ELSE = 311,
    T_EMPTY = 312,
    T_EMPTY_SEQUENCE = 313,
    T_ENCODING = 314,
    T_END_SORT = 315,
    T_EQ = 316,
    T_ERROR = 317,
    T_EVERY = 318,
    T_EXCEPT = 319,
    T_EXTERNAL = 320,
    T_FOLLOWING = 321,
    T_FOLLOWING_SIBLING = 322,
    T_FOLLOWS = 323,
    T_FOR_APPLY_TEMPLATE = 324,
    T_FOR = 325,
    T_FUNCTION = 326,
    T_GE = 327,
    T_G_EQ = 328,
    T_G_GE = 329,
    T_G_GT = 330,
    T_G_LE = 331,
    T_G_LT = 332,
    T_G_NE = 333,
    T_GREATEST = 334,
    T_GT = 335,
    T_IDIV = 336,
    T_IF = 337,
    T_IMPORT = 338,
    T_INHERIT = 339,
    T_IN = 340,
    T_INSTANCE = 341,
    T_INTERSECT = 342,
    T_IS = 343,
    T_ITEM = 344,
    T_LAX = 345,
    T_LBRACKET = 346,
    T_LEAST = 347,
    T_LE = 348,
    T_LET = 349,
    T_LPAREN = 350,
    T_LT = 351,
    T_MAP = 352,
    T_MATCHES = 353,
    T_MINUS = 354,
    T_MODE = 355,
    T_MOD = 356,
    T_MODULE = 357,
    T_NAME = 358,
    T_NAMESPACE = 359,
    T_NE = 360,
    T_NODE = 361,
    T_NO_INHERIT = 362,
    T_NO_PRESERVE = 363,
    T_OF = 364,
    T_OPTION = 365,
    T_ORDERED = 366,
    T_ORDERING = 367,
    T_ORDER = 368,
    T_OR = 369,
    T_PARENT = 370,
    T_PI_START = 371,
    T_PLUS = 372,
    T_POSITION_SET = 373,
    T_PRAGMA_END = 374,
    T_PRAGMA_START = 375,
    T_PRECEDES = 376,
    T_PRECEDING = 377,
    T_PRECEDING_SIBLING = 378,
    T_PRESERVE = 379,
    T_PRIORITY = 380,
    T_PROCESSING_INSTRUCTION = 381,
    T_QUESTION = 382,
    T_QUICK_TAG_END = 383,
    T_QUOTE = 384,
    T_RBRACKET = 385,
    T_RETURN = 386,
    T_RPAREN = 387,
    T_SATISFIES = 388,
    T_SCHEMA_ATTRIBUTE = 389,
    T_SCHEMA_ELEMENT = 390,
    T_SCHEMA = 391,
    T_SELF = 392,
    T_SEMI_COLON = 393,
    T_SLASH = 394,
    T_SLASHSLASH = 395,
    T_SOME = 396,
    T_SORT = 397,
    T_STABLE = 398,
    T_STAR = 399,
    T_STRICT = 400,
    T_STRIP = 401,
    T_SUCCESS = 402,
    T_COMMENT_CONTENT = 403,
    T_PI_CONTENT = 404,
    T_PI_TARGET = 405,
    T_XSLT_VERSION = 406,
    T_TEMPLATE = 407,
    T_TEXT = 408,
    T_THEN = 409,
    T_TO = 410,
    T_TREAT = 411,
    T_TUNNEL = 412,
    T_TYPESWITCH = 413,
    T_UNION = 414,
    T_UNORDERED = 415,
    T_VALIDATE = 416,
    T_VARIABLE = 417,
    T_VERSION = 418,
    T_WHERE = 419,
    T_XQUERY = 420,
    T_INTERNAL = 421,
    T_INTERNAL_NAME = 422,
    T_CURRENT = 423
  };
#endif

/* Value type.  */

/* Location type.  */
#if ! defined YYLTYPE && ! defined YYLTYPE_IS_DECLARED
typedef struct YYLTYPE YYLTYPE;
struct YYLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
};
# define YYLTYPE_IS_DECLARED 1
# define YYLTYPE_IS_TRIVIAL 1
#endif



int XPathparse (QT_PREPEND_NAMESPACE(QPatternist)::ParserContext *const parseInfo);

#endif /* !YY_XPATH_QQUERYTRANSFORMPARSER_P_H_INCLUDED  */
