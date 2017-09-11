<!-- Locked by waynegramlich on 2017Sep9 to do initial development. -->

# Ubiquity Robotics Documentation Standard

This document describes the Ubiquity Robotics documentation formatting
and writing standards.  At this point in time, this is a proposal that can be
discussed and changed.  This document is broken into the following sections:

1. Formatting Guidelines

2. Approved Formatting Extensions

3. Writing Style

4. Documentation Workflow

## Formatting Guidelines

Some formatting guidelines are listed immediately below:

* Comments:
  Non-visible comments can be inserted into the document using standard HTML comments
  of the following form:

          <!--  Comment goes here. -->

* Description Lists:
  [HTML description lists](https://www.w3schools.com/tags/tag_dl.asp) are not
  supported by Markdown.  We improvise around the lack of this functionality by
  using lists as shown immediately below:

  >        * Description Word or Phrase:
  >          First paragraph of description...
  >          ...
  >          N'th paragraph of description...

  This list uses this style of description lists.

* Directory Names:
  If the directory is named `foo`, the markdown file should be named `foo.md`.
  Directory names should somewhat descriptive without getting overly long.
  For now, directory names should be somewhere be 10 to 25 characters long.
  Note that this is a guideline rather than a hard and fast rule.  Do not label
  directories with chapter numbers (e.g. `chapter1`, `chapter2`, etc.)
  Directory names should be all lower case with underscores ('_') between words.
  Long words like "documentation" and "miscellaneous" can be shortened down to
  "doc" and "misc", respectively.  Do not do do vowel removal.  Thus, shortening
  "documentation"  and "miscellaneous" down to "dcmnttn" and "mscllns" is not
  allowed.

* Directory Per Page:
  Each Markdown document should be in its own directory.
  Since markdown provides a way to include images, almost all images associated
  with the page are colocated in the directory containing the markdown page.
  This means when the page is no longer needed, the entire directory can be deleted
  along with all of its associated images.

* Global Images Directory:
  Some images are used over and over again.  This images are stored in a
  global top level directory named `assets/`.  Do not go wild stuffing images
  into the `assets` directory.  Each image that goes goes in there should
  be an obviously sharable image.  Some examples are `loki.jpg`, `magni.jpg`,
  `robots.jpg`, etc.

* Images:
  Pictures are saved with a suffix of `.jpg`.  Screen snapshots are saved with
  as suffix of `.png`.  The Markdown syntax for an image is:

  >        \![Alternative Text][URL]

  Please put something in alternative text to describe the image.  URL's
  should always use relative syntax.  Most of the images are colocated in
  the same directory, so the URL's have the form of `image_name.jpg` or
  `image_name.png`.

* Line Wrapping:
  Currently, there is no standard for line wrapping.  If you like your paragraphs
  to reside on a single line, that is quite acceptable.  Breaking a paragraph
  so that there is line wrapping is quite acceptable.  The arbitrary buffer
  width of 100 characters is assumed for line chopping.  Markdown link and image
  syntax can not be broken across multiple lines, so some line wrapping is unavoidable.

* Links:
  The format for a link is:

  >        ![Link Text](Link URL)

  Try to embed the link into a sentence.  Do not use the cheat of "click here"
  for link text.   If the URL is to a page local to the repository, use relative
  links.  If the URL is to another web site or repository, use a full URL.

* Table of Contents:
  Sometimes a page is just a list of links to other pages.  The format for this 
  show below:

  >        {overview paragraphs}
  >
  >        * [Short Description1](Relative URL1):
  >          Longer description.
  >        
  >          ...
  >        
  >        * [Short DescriptionN](Relative URLn):
  >          Longer description.
  >
  >        {optional conclusion paragraphs}

## Approved Formatting Extensions

In general, the documentation is written in standard
[Markdown format](https://daringfireball.net/projects/markdown/syntax).

The current intention is to seriously consider using
[GitHub Pages](https://pages.github.com/).
Do not use any GitHub Markdown extensions unless they are listed below.
Github Pages provide some extensions to Markdown format.

Additionally, Github Pages uses
[Jekyll](https://jekyllrb.com/docs/home/).
We may decide to incorporate some Jekyll features into the UR formatting
standards.

Only use Markdown and Jeykll features that have been agreed upon and
incorporated into Ubiquity Robotics documentation.

The acceptable GitHub Markdown extensions are:

* Currently, there are no agreed upon Github Markdown extensions.

The acceptable Jekyll extensions are:

* There are currently no agreed upon Jekyll extensions.

## Writing Style

Documentation is easier for the customer to read if everybody uses a similar
writing style.  The list of writing style recommendations is listed 
alphabetically below.

* Acronyms:
  The first time you use an acronym on a page, follow the acronym with
  a parenthetical expansion of the phase with the appropriate letters capitalizes.
  You may use the UR (Ubiquity Robotics) without having to do the expansion.

* Dates:
  Calendar dates are always represented in `YYYYMMMDD` format where:

  * `YYYYY` is 4-digit year,

  * `MMM` is a 3 character month (i.e. `Jan`, `Feb`, ..., `Dec`.)

  * `DD` is the day of the month (i.e. `01`, `02`, ..., `31`.)

* Embedded Defect/Issue Reports:
  Documentation and associated software defect/issue reports are embedded
  directly into the documentation using the format below:

        *{ GIT_USER_NAME: DESCRIPTION ... }*

  where:
 
  * `GIT_USER_NAME`:
    This is the GitHub user account name of the person reporting the defect.


  * `DESCRIPTION`:
    `DESCRIPTION` is a description of the defect.

  These defect/issues reports are inserted into the documentation as a separate
  markdown paragraph that immediately following a documentation paragraph that
  that has the defect/issue.

  Read the *Documentation Workflow* section below for more information about
  defect/issue reports.

* No Surprises:
  Writing technical documentation is not like writing the great American novel.
  In particular, the customer should not be left wondering where the documentation
  is going.  A strategy of providing a brief preview, followed by the details, and
  finishing with a conclusion is both simple and effective.

* Pronouns:
  Documentation tends to be easier to read if it is documentation team having
  a conversation with the customers.  Thus, use "we" and "us" to refer to the
  UR development team and "you" and "your" to refer to the customers.

* Robot Names:
  Only use specific robot names when talking about a feature that is specific
  to that robot.  Thus, "Magni" and "Loki" are to be used only features that
  are specific to the Magni and Loki robot platforms.  Use the phrase "your robot"
  to refer to a generic robot (i.e. the customer's robot.)  This allows Ubiquity
  Robots to add a new platform without having to do a massive edit.

* Tense:
  The dominate tense to use is the present tense.  Only use the past tense
  to document deprecated features.  Only use future tense infrequently, and
  then only to describe features that have not been implemented and are not
  likely to be implemented soon.

  If you are documenting a feature that has not been implemented yet, but will
  be implemented relatively soon, please write the documentation in the present
  tense.  Some sort of marker is needed to flag documented features that have
  not been implemented yet.  For now, just use `Not Implemented:` at the
  beginning of each paragraph or section that is not implemented.

* Textual References:
  Sometimes you need to refer to some text elsewhere in the page.  Back references
  text earlier in the page should use the term "above".  Forward references to
  text later in the page, should use the term "below".  The term "immediately below"
  means that the text shows up within a paragraph or two.  The term "much further below"
  means that the text shows up many paragraphs below.  Do not fall into the trap
  using past/future tense instead of present tense.  Do not say "This *was* explained
  earlier in this document" or "This will be explained later in this document."  Always
  use present tense "As explained above ..." and "The code snippet immediately below
  shows how to ..."

* Use Lists and Tables:
  Use lists and tables to organize information.  Currently, Markdown supports lists
  pretty well.  Tables currently have to be done using HTML tags.

## Documentation Workflow

Documentation workflow is the term being used to describe how documentation gets
developed and improved over time.

GitHub has developed an integrated bug/issue/feature/task/defect tracking system.
This is the same basic system that UR uses track bugs and add features to UR code
base.  GitHub tends to use the term "issue" for bugs/issues/defect/task/defects.
In general, the term is "defect" is slightly more descriptive term in that it indicates
some action needs to happen before the "defect" is resolved and removed.  Regardless,
of the terminology, the goal is to massively leverage GitHub issue tracker to manage
the documentation workflow.

It is really important to encourage the people who read the Ubiquity Robotics
documentation to submit defect/issue reports on the UR documentation.  This helps to
build an active user community that is actively involved in overall documentation
improvement.

Embedded Issue/Defect Reports (see above) are part of ...

The basic workflow is:

1. While reading the documentation a defect or issue is identified.

2. The reader clicks on the [View on GitHub] button that navigates to the GitHub
   page that contains the content.  (Actually, this does not work right now.
   Does Jekyll doe anything that is actually usefue?)

3. The reader clicks on the little pencil that opens the errant page in a web broser.

4. The reader quickly edits in a simple embedded 

4. The reader submits their change as a pull request in the case of their documentation change having a code implication or if it simply needs review and discussion. The overwhelming majority of documentation changes, however should just be committed directly to the master branch. We can revert changes that proove unpopular.

5. From this point forward, the standard GitHub issue tracking system takes over.
   
A couple of other issue:

* Typographical and Grammatical Errors:
  Members of the UR team should fix these immediately on the GitHub site.
  Use the comment "Fixed some typos." to mark these.

* Locking:
  When somebody is going to do a massive reedit of a page, you should "lock"
  the page to discourage people from mucking with the page.  Locking is performed
  with a comment of the following form at the head of the page:

  >        \<!-- Locked by USER_NAME on YYYYMMMDD for REASON. -->

    * USER_NAME is the github user name for the person doing the locking.

    * YYYYMMMDD is standard *Date* format.
      
    * REASON is a very short reason why the page is locked.
    
## Conclusion

*{ waynegramlich: The conclusion is missing.}*
