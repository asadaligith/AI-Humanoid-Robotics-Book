# Review Process Workflow

**Document Type**: Process Documentation
**Last Updated**: 2025-12-20
**Status**: Active

---

## Overview

This document defines the review process for all content, code, and deliverables in the "Physical AI & Humanoid Robotics" educational platform. The process ensures quality, consistency, and adherence to project standards before production deployment.

---

## Review Roles and Responsibilities

### 1. Content Author
**Who**: Technical writer, developer, or subject matter expert creating content

**Responsibilities**:
- Create content following project templates and constitution
- Self-review against quality gates before submission
- Run automated validation tools (linter, tests, build)
- Address reviewer feedback promptly
- Update content based on review comments

### 2. Technical Reviewer (Subject Matter Expert)
**Who**: Experienced robotics engineer or researcher (2+ years ROS 2)

**Qualifications**:
- Deep knowledge of ROS 2 Humble, robotics simulation, humanoid systems
- Familiarity with Isaac Sim, Gazebo, Nav2, MoveIt 2
- Understanding of VLA (Vision-Language-Action) pipelines

**Responsibilities**:
- Verify technical accuracy of all claims and code
- Check citations and source quality
- Validate code examples (test if possible)
- Review difficulty appropriateness for target audience
- Provide constructive feedback within 2 business days

### 3. Accessibility Reviewer
**Who**: Frontend developer with WCAG knowledge OR accessibility specialist

**Qualifications**:
- WCAG 2.1 Level AA certification (preferred) or demonstrated knowledge
- Experience with Lighthouse, axe DevTools, screen readers

**Responsibilities**:
- Run automated accessibility audits (Lighthouse, axe)
- Test keyboard navigation
- Test with screen reader (sample pages)
- Verify color contrast ratios
- Provide remediation guidance for violations

### 4. Pedagogical Reviewer
**Who**: Education specialist, instructor, or program manager

**Qualifications**:
- Understanding of instructional design principles
- Familiarity with Bloom's taxonomy
- Experience with technical education (preferred)

**Responsibilities**:
- Validate learning objectives clarity and measurability
- Check progressive complexity and prerequisite alignment
- Review exercise relevance to learning outcomes
- Verify chapter structure consistency
- Assess overall pedagogical effectiveness

### 5. Deployment Reviewer (DevOps/Technical Writer)
**Who**: DevOps engineer or technical writer

**Responsibilities**:
- Verify markdown formatting correctness
- Run link checker (broken-link-checker)
- Test Docusaurus build
- Check for placeholders/TODOs
- Visual inspection of deployed content

### 6. Approver (Program Manager or Lead)
**Who**: Program manager or technical lead

**Responsibilities**:
- Final approval for production deployment
- Resolve conflicts between reviewers
- Approve quality gate exceptions
- Monitor overall project quality metrics

---

## Review Workflow

### Phase 1: Author Preparation (1-2 hours per chapter)

1. **Create Content**
   - Follow chapter template structure
   - Adhere to word count limits (700-1500 words)
   - Include citations for technical claims
   - Write code examples with comments

2. **Self-Review Checklist**
   - [ ] Learning objectives stated clearly
   - [ ] All technical claims cited (min 2 sources)
   - [ ] Code tested and runs without errors
   - [ ] Word count within 700-1500 range
   - [ ] Markdown formatting correct
   - [ ] No placeholders or TODOs
   - [ ] Images have alt text

3. **Automated Validation**
   ```bash
   # Run linter
   npx markdownlint docs/modules/**/*.md

   # Check word count
   wc -w docs/modules/module-01-ros2-fundamentals/chapter-02-pubsub.md

   # Test build
   npm run build

   # Check links
   npx broken-link-checker http://localhost:3000 --recursive
   ```

4. **Submit for Review**
   - Create pull request (PR) with descriptive title
   - Assign reviewers: Technical, Accessibility, Pedagogical, Deployment
   - Include self-review checklist in PR description
   - Link to related issues/tasks

---

### Phase 2: Technical Review (4-8 hours per module)

**Timeline**: Within 2 business days of submission

**Reviewer Actions**:

1. **Accuracy Verification**
   - [ ] All technical claims have authoritative citations
   - [ ] Citations link to correct sources (not 404)
   - [ ] Technical terminology accurate (e.g., "topic" not "channel" in ROS 2)
   - [ ] Version specificity correct (ROS 2 Humble, not generic ROS 2)

2. **Code Validation**
   - [ ] Copy code snippets to isolated environment
   - [ ] Execute code and verify expected output
   - [ ] Check for syntax errors, deprecated APIs
   - [ ] Validate best practices (error handling, logging)

3. **Conceptual Review**
   - [ ] Explanations technically sound
   - [ ] Diagrams accurately represent concepts
   - [ ] Difficulty appropriate for intermediate-advanced learners
   - [ ] Prerequisites correctly stated

4. **Feedback Submission**
   - Use PR comments for inline feedback
   - Categorize issues: Critical (must fix), Important (should fix), Suggestion (nice-to-have)
   - Provide constructive alternatives where applicable
   - Approve or request changes within 2 days

**Approval Criteria**:
- ✅ APPROVE: Zero critical issues, max 2 important issues
- ⚠️ REQUEST CHANGES: 1+ critical issues OR 3+ important issues
- 🚫 REJECT: Fundamental conceptual errors, require complete rewrite

---

### Phase 3: Accessibility Review (2-4 hours per module)

**Timeline**: Can run in parallel with Technical Review

**Reviewer Actions**:

1. **Automated Audit**
   ```bash
   # Run Lighthouse
   npx lighthouse http://localhost:3000/docs/path --view

   # Run axe DevTools
   npx axe http://localhost:3000/docs/path
   ```

2. **Keyboard Navigation Test**
   - [ ] Tab through all interactive elements
   - [ ] Verify focus indicators visible
   - [ ] Test keyboard shortcuts (e.g., Ctrl+K for search)
   - [ ] No keyboard traps (can escape all modals)

3. **Screen Reader Test**
   - [ ] Install NVDA (Windows) or VoiceOver (macOS)
   - [ ] Navigate through chapter headings
   - [ ] Verify all images have descriptive alt text
   - [ ] Check code blocks are readable

4. **Color Contrast Check**
   - [ ] Use WebAIM Contrast Checker
   - [ ] Verify normal text: 4.5:1 minimum
   - [ ] Verify large text: 3:1 minimum
   - [ ] Check interactive elements: 3:1 minimum

**Approval Criteria**:
- ✅ APPROVE: Lighthouse accessibility score ≥90, zero critical violations
- ⚠️ REQUEST CHANGES: Score &lt;90 OR critical violations found
- 📋 EXCEPTION: If third-party components cause violations, document exception

---

### Phase 4: Pedagogical Review (2-4 hours per module)

**Timeline**: Can run in parallel with Technical/Accessibility Reviews

**Reviewer Actions**:

1. **Learning Objectives Validation**
   - [ ] Objectives use measurable action verbs (create, analyze, apply, NOT understand/know)
   - [ ] Objectives align with module-level outcomes
   - [ ] 3-5 objectives per chapter (not too few/many)

2. **Progressive Complexity Check**
   - [ ] Concepts build on previous chapters
   - [ ] Prerequisites explicitly stated
   - [ ] Difficulty appropriate for target audience (intermediate-advanced)
   - [ ] Advanced topics flagged as optional

3. **Exercise Validation**
   - [ ] At least 1 hands-on exercise per chapter
   - [ ] Exercises test specific learning objectives
   - [ ] Exercise solvable with chapter content
   - [ ] Real-world application demonstrated

4. **Structure Consistency**
   - [ ] Chapter follows template (Introduction, Concepts, Implementation, Exercises, Summary)
   - [ ] Word count within 700-1500 range
   - [ ] Consistent tone and style across chapters

**Approval Criteria**:
- ✅ APPROVE: All criteria met, effective learning experience
- ⚠️ REQUEST CHANGES: Learning objectives unclear OR exercises don't test outcomes
- 🚫 REJECT: Difficulty inappropriate OR missing critical structure elements

---

### Phase 5: Deployment Readiness Review (1-2 hours)

**Timeline**: After Technical/Accessibility/Pedagogical approvals

**Reviewer Actions**:

1. **Markdown Formatting Validation**
   ```bash
   # Lint markdown
   npx markdownlint docs/modules/module-01-ros2-fundamentals/*.md
   ```
   - [ ] Frontmatter YAML valid (title, description, sidebar_position)
   - [ ] Headings properly nested (# → ## → ###, no skips)
   - [ ] Code blocks have language specified
   - [ ] Internal links use correct format

2. **Link Validation**
   ```bash
   # Check all links
   npx broken-link-checker http://localhost:3000 --recursive --filter-level 3
   ```
   - [ ] No 404 errors (broken internal links)
   - [ ] External links resolve (HTTP 200)
   - [ ] Anchor links point to existing headings

3. **Placeholder Scan**
   ```bash
   # Search for incomplete content markers
   grep -rn "TODO\|FIXME\|XXX\|TBD\|\[placeholder\]" docs/modules/module-01-ros2-fundamentals/
   ```
   - [ ] No TODO/FIXME/XXX markers
   - [ ] No [placeholder] text
   - [ ] All content finalized

4. **Build Test**
   ```bash
   # Production build
   npm run build

   # Serve locally for inspection
   npm run serve
   ```
   - [ ] Build completes with zero errors
   - [ ] No warnings about missing files
   - [ ] Bundle size reasonable (&lt;50MB)

5. **Visual Inspection**
   - [ ] Chapter renders correctly in browser
   - [ ] Images load properly
   - [ ] Code blocks formatted correctly
   - [ ] Navigation functional (sidebar, next/prev)

**Approval Criteria**:
- ✅ APPROVE: Build succeeds, zero broken links, no placeholders
- ⚠️ REQUEST CHANGES: Build warnings OR broken links found
- 🚫 REJECT: Build fails

---

### Phase 6: Final Approval and Deployment (30 minutes)

**Approver Actions**:

1. **Review Summary**
   - Verify all 4 quality gates passed (Technical, Accessibility, Pedagogical, Deployment)
   - Check that all reviewer feedback addressed
   - Resolve any conflicts between reviewers

2. **Approve PR**
   - Merge pull request to `main` branch
   - Ensure commit message follows convention:
     ```
     docs(module-01): add Chapter 2 - Pub/Sub Pattern

     - Add learning objectives for publisher-subscriber pattern
     - Include code examples for topic communication
     - Add exercise for sensor data logging

     Closes #123
     ```

3. **Monitor Deployment**
   - GitHub Actions workflow triggers automatically
   - Verify deployment succeeds (check Actions tab)
   - Smoke test live site: https://asadaligith.github.io/AI-Humanoid-Robotics-Book/

4. **Post-Deployment Validation**
   - [ ] Chapter accessible on live site
   - [ ] Navigation works (sidebar shows new chapter)
   - [ ] Links functional
   - [ ] Images load correctly
   - [ ] ChatGPT widget still functional (if integrated)

---

## Review Timeline Expectations

| Review Type | Expected Turnaround | Escalation If Delayed |
|-------------|---------------------|----------------------|
| Technical Review | 2 business days | Notify program manager after 3 days |
| Accessibility Review | 2 business days | Notify program manager after 3 days |
| Pedagogical Review | 2 business days | Notify program manager after 3 days |
| Deployment Review | 1 business day | Notify DevOps lead after 2 days |
| Final Approval | 1 business day | Notify program manager after 2 days |

**Total Expected Timeline**: 5-7 business days (assuming parallel reviews)

---

## Feedback Guidelines

### For Reviewers

**DO**:
- Be specific (cite line numbers, provide examples)
- Be constructive (suggest alternatives, not just criticize)
- Prioritize issues (Critical > Important > Suggestion)
- Explain reasoning (why something is incorrect or problematic)
- Acknowledge good work (positive reinforcement)

**DON'T**:
- Use vague language ("this section needs work" without specifics)
- Make personal attacks or dismissive comments
- Block on style preferences (unless violates standards)
- Request changes outside project scope
- Delay feedback beyond expected timeline

### For Authors

**DO**:
- Respond to all feedback points (even if declining suggestion)
- Ask clarifying questions if feedback unclear
- Make requested changes promptly (within 1-2 days)
- Mark resolved comments (GitHub "Resolve conversation")
- Notify reviewers when ready for re-review

**DON'T**:
- Ignore reviewer comments without explanation
- Argue defensively (assume good intent)
- Make partial fixes (address all issues before re-submitting)
- Sneak in unrelated changes (keep PRs focused)
- Rush through revisions (quality over speed)

---

## Exception Handling

### Quality Gate Cannot Be Met

If a quality gate cannot be passed due to technical constraints:

1. **Document Exception Request**
   - Which gate cannot be met
   - Reason (technical constraint, third-party dependency, etc.)
   - Proposed mitigation (alternative approach)
   - Impact assessment (how does this affect users?)

2. **Approval Process**
   - Submit exception request to program manager
   - Program manager reviews with technical lead
   - Decision within 1 business day
   - Document approval in exceptions log

3. **Track Exception**
   - Add to `docs/exceptions-log.md`
   - Include resolution plan (when can this be fixed?)
   - Schedule review (quarterly check for resolution)

### Reviewer Conflict

If reviewers disagree on approval decision:

1. **Reviewers discuss** (async via PR comments or sync via meeting)
2. **Program manager mediates** if no consensus after 1 day
3. **Final decision** by program manager or technical lead
4. **Document decision** in PR comments for transparency

### Urgent Hotfix

For critical bugs requiring immediate fix:

1. **Create hotfix PR** with `[URGENT]` prefix
2. **Fast-track review** (4-hour turnaround expected)
3. **Skip non-critical gates** (e.g., pedagogical review for bug fix)
4. **Deploy immediately** after technical + deployment reviews
5. **Backfill skipped reviews** within 1 week

---

## Continuous Improvement

### Quarterly Review Process Audit

Every quarter (January, April, July, October):

1. **Collect Metrics**
   - Average review turnaround time
   - Number of rejected PRs
   - Most common review issues
   - Reviewer workload balance

2. **Analyze Bottlenecks**
   - Identify slow stages (Technical? Accessibility?)
   - Root cause analysis (lack of reviewers? unclear standards?)

3. **Update Process**
   - Refine review criteria based on learnings
   - Adjust timelines if consistently unmet
   - Add new automated checks to reduce manual load

4. **Train Reviewers**
   - Share common issues and resolutions
   - Update reviewer guidelines
   - Onboard new reviewers as project scales

---

## Tools and Resources

### Automated Validation Tools

```bash
# Install tools
npm install -g markdownlint-cli broken-link-checker lighthouse axe-cli

# Lint markdown
npx markdownlint docs/modules/**/*.md

# Check links
npx broken-link-checker http://localhost:3000 --recursive

# Accessibility audit
npx lighthouse http://localhost:3000/docs/path --view
npx axe http://localhost:3000/docs/path

# Word count
wc -w docs/modules/module-01-ros2-fundamentals/chapter-02-pubsub.md

# Build test
npm run build
```

### Review Templates

- **PR Description Template**: `.github/PULL_REQUEST_TEMPLATE.md`
- **Technical Review Checklist**: `docs/review-checklists/technical-review.md`
- **Accessibility Review Checklist**: `docs/review-checklists/accessibility-review.md`
- **Pedagogical Review Checklist**: `docs/review-checklists/pedagogical-review.md`

### Reference Documents

- **Quality Gates**: `docs/quality-gates.md`
- **Constitution**: `.specify/memory/constitution.md`
- **Chapter Template**: `docs/templates/chapter-template.md` (if exists)
- **Style Guide**: `docs/style-guide.md` (if exists)

---

## Revision History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-12-20 | 1.0 | Initial review process documentation | AI Assistant |

---

**Status**: ✅ Active
**Next Review**: 2026-01-20 (monthly review)
