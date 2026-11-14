//! Every question `stela` asks, behind one interface: the terminal's
//! Clack-style prompts ([`Terminal`]), none at all for scripts
//! ([`NoTerminal`]), and in tests a script of answers that records what was
//! shown (`Scripted`). Each prompt can be declined: Escape or Control+C
//! cancels at a terminal, and every question names a way out.
use super::wizard::menu::{self, Item};
use anyhow::{Result, bail};
use std::io::IsTerminal;

/// A check of typed text: the reason it is refused.
pub(super) type Validate = Box<dyn Fn(&str) -> Result<(), String>>;

pub(super) trait Ui {
    /// Whether someone can answer: a terminal on standard input and error.
    fn interactive(&self) -> bool;
    fn intro(&self, title: &str) -> Result<()>;
    fn info(&self, text: &str) -> Result<()>;
    fn warning(&self, text: &str) -> Result<()>;
    fn note(&self, title: &str, text: &str) -> Result<()>;
    /// One of `items` by its value; the enabled one at `initial` first.
    fn select(&self, prompt: &str, items: &[Item], initial: usize) -> Result<usize>;
    /// A line of text, `default` filled in.
    fn input(
        &self,
        prompt: &str,
        placeholder: &str,
        default: &str,
        valid: Validate,
    ) -> Result<String>;
    /// A line of text that is not shown.
    fn password(&self, prompt: &str, valid: Validate) -> Result<String>;
    /// Yes or no; no unless answered.
    fn confirm(&self, question: &str) -> Result<bool>;
    fn outro(&self, text: &str) -> Result<()>;
    /// The end of a run that changed nothing.
    fn cancel(&self, text: &str) -> Result<()>;
}

/// The menu rows of `choices`, each enabled, valued by position.
pub(super) fn rows(choices: &[(&str, &str)]) -> Vec<Item> {
    choices
        .iter()
        .enumerate()
        .map(|(value, (label, hint))| Item {
            value,
            label: (*label).into(),
            hint: (*hint).into(),
            enabled: true,
        })
        .collect()
}

/// Whether `error` is a prompt the user cancelled.
pub(super) fn cancelled(error: &anyhow::Error) -> bool {
    error
        .downcast_ref::<std::io::Error>()
        .is_some_and(|e| e.kind() == std::io::ErrorKind::Interrupted)
}

/// The terminal's prompts, in Stela's Clack theme.
pub(super) struct Terminal;

/// Whether standard input and error are a terminal: someone can answer.
pub(super) fn attached() -> bool {
    std::io::stdin().is_terminal() && std::io::stderr().is_terminal()
}

impl Terminal {
    /// The terminal when standard input and error are one, else no one.
    pub(super) fn or_none() -> &'static dyn Ui {
        if attached() {
            cliclack::set_theme(super::wizard::theme::Stela);
            &Terminal
        } else {
            &NoTerminal
        }
    }
}

impl Ui for Terminal {
    fn interactive(&self) -> bool {
        true
    }
    fn intro(&self, title: &str) -> Result<()> {
        crate::progress::follow(super::wizard::theme::rail());
        Ok(cliclack::intro(
            super::wizard::theme::accent().bold().apply_to(title),
        )?)
    }
    fn info(&self, text: &str) -> Result<()> {
        Ok(cliclack::log::info(text)?)
    }
    fn warning(&self, text: &str) -> Result<()> {
        Ok(cliclack::log::warning(text)?)
    }
    fn note(&self, title: &str, text: &str) -> Result<()> {
        Ok(cliclack::note(title, text)?)
    }
    fn select(&self, prompt: &str, items: &[Item], initial: usize) -> Result<usize> {
        menu::select(prompt, items, initial)
    }
    fn input(
        &self,
        prompt: &str,
        placeholder: &str,
        default: &str,
        valid: Validate,
    ) -> Result<String> {
        Ok(cliclack::input(prompt)
            .placeholder(placeholder)
            .default_input(default)
            .validate(move |text: &String| valid(text))
            .interact()?)
    }
    fn password(&self, prompt: &str, valid: Validate) -> Result<String> {
        Ok(cliclack::password(prompt)
            .mask('•')
            .validate(move |text: &String| valid(text))
            .interact()?)
    }
    fn confirm(&self, question: &str) -> Result<bool> {
        Ok(cliclack::confirm(question)
            .initial_value(false)
            .interact()?)
    }
    fn outro(&self, text: &str) -> Result<()> {
        Ok(cliclack::outro(text)?)
    }
    fn cancel(&self, text: &str) -> Result<()> {
        Ok(cliclack::outro_cancel(text)?)
    }
}

/// No one to ask: scripts, CI and piped input. Every question is refused,
/// so a command stops before it changes anything it would ask about.
pub(super) struct NoTerminal;

impl Ui for NoTerminal {
    fn interactive(&self) -> bool {
        false
    }
    fn intro(&self, _: &str) -> Result<()> {
        Ok(())
    }
    fn info(&self, text: &str) -> Result<()> {
        eprintln!("{text}");
        Ok(())
    }
    fn warning(&self, text: &str) -> Result<()> {
        eprintln!("{text}");
        Ok(())
    }
    fn note(&self, _: &str, text: &str) -> Result<()> {
        eprintln!("{text}");
        Ok(())
    }
    fn select(&self, prompt: &str, _: &[Item], _: usize) -> Result<usize> {
        bail!("{prompt}: stela cannot ask without a terminal")
    }
    fn input(&self, prompt: &str, _: &str, _: &str, _: Validate) -> Result<String> {
        bail!("{prompt}: stela cannot ask without a terminal")
    }
    fn password(&self, prompt: &str, _: Validate) -> Result<String> {
        bail!("{prompt}: stela cannot ask without a terminal")
    }
    fn confirm(&self, question: &str) -> Result<bool> {
        bail!("{question} stela cannot ask without a terminal")
    }
    fn outro(&self, text: &str) -> Result<()> {
        eprintln!("{text}");
        Ok(())
    }
    fn cancel(&self, text: &str) -> Result<()> {
        eprintln!("{text}");
        Ok(())
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use std::{cell::RefCell, collections::VecDeque};

    /// What happens on the computer once a question is answered.
    type Hook<'a> = Option<Box<dyn Fn(&str) + 'a>>;

    /// The answer that presses Enter.
    pub(crate) const ENTER: &str = "Enter";

    /// A person at a terminal who gives these answers in order, and what
    /// they saw: each prompt as `? prompt`, each choice as `● label · hint`
    /// for the one chosen first and `○ label · hint` for the others
    /// (`(unavailable)` when greyed out), each message as it was shown.
    /// Answers name a choice by its label, or are the text typed, or `yes`
    /// and `no`; `cancel` presses Escape and [`ENTER`] takes what the
    /// question offers first: the marked choice, the filled-in text, no.
    pub(crate) struct Scripted<'a> {
        answers: RefCell<VecDeque<String>>,
        pub(crate) shown: RefCell<Vec<String>>,
        hook: Hook<'a>,
    }

    impl<'a> Scripted<'a> {
        pub(crate) fn new(answers: &[&str]) -> Self {
            Self {
                answers: RefCell::new(answers.iter().map(|a| (*a).to_owned()).collect()),
                shown: RefCell::default(),
                hook: None,
            }
        }

        /// The same person, and `hook` run with each answer they give.
        pub(crate) fn with_hook(answers: &[&str], hook: impl Fn(&str) + 'a) -> Self {
            Self {
                hook: Some(Box::new(hook)),
                ..Self::new(answers)
            }
        }

        /// Everything shown, one line each.
        pub(crate) fn transcript(&self) -> String {
            self.shown.borrow().join("\n")
        }

        /// Whether every answer was taken.
        pub(crate) fn done(&self) -> bool {
            self.answers.borrow().is_empty()
        }

        fn show(&self, line: String) {
            self.shown.borrow_mut().push(line);
        }

        fn answer(&self, prompt: &str) -> Result<String> {
            let answer =
                self.answers.borrow_mut().pop_front().unwrap_or_else(|| {
                    panic!("no answer left for {prompt:?}:\n{}", self.transcript())
                });
            if answer == "cancel" {
                return Err(
                    std::io::Error::new(std::io::ErrorKind::Interrupted, "cancelled").into(),
                );
            }
            if let Some(hook) = &self.hook {
                hook(&answer);
            }
            Ok(answer)
        }
    }

    impl Ui for Scripted<'_> {
        fn interactive(&self) -> bool {
            true
        }
        fn intro(&self, title: &str) -> Result<()> {
            self.show(format!("┌ {title}"));
            Ok(())
        }
        fn info(&self, text: &str) -> Result<()> {
            self.show(format!("● {text}"));
            Ok(())
        }
        fn warning(&self, text: &str) -> Result<()> {
            self.show(format!("▲ {text}"));
            Ok(())
        }
        fn note(&self, title: &str, text: &str) -> Result<()> {
            self.show(format!("◇ {title}"));
            for line in text.lines() {
                self.show(format!("  {line}"));
            }
            Ok(())
        }
        fn select(&self, prompt: &str, items: &[Item], initial: usize) -> Result<usize> {
            self.show(format!("? {prompt}"));
            // As the menu does: the enabled choice valued `initial`, else
            // the first enabled one.
            let first = items
                .iter()
                .find(|item| item.enabled && item.value == initial)
                .or_else(|| items.iter().find(|item| item.enabled))
                .map(|item| item.value);
            for item in items {
                let mark = if Some(item.value) == first {
                    '●'
                } else {
                    '○'
                };
                let mut line = format!("{mark} {}", item.label);
                if !item.hint.is_empty() {
                    line.push_str(&format!(" · {}", item.hint));
                }
                if !item.enabled {
                    line.push_str(" (unavailable)");
                }
                self.show(line);
            }
            let answer = self.answer(prompt)?;
            // Enter takes the choice marked first.
            if answer == ENTER {
                return Ok(first.expect("a menu has an enabled choice"));
            }
            let item = items
                .iter()
                .find(|item| item.label == answer)
                .unwrap_or_else(|| {
                    panic!("{prompt:?} offers no {answer:?}:\n{}", self.transcript())
                });
            assert!(
                item.enabled,
                "{answer:?} is unavailable:\n{}",
                self.transcript()
            );
            Ok(item.value)
        }
        fn input(&self, prompt: &str, _: &str, default: &str, valid: Validate) -> Result<String> {
            self.show(format!("? {prompt}"));
            let mut answer = self.answer(prompt)?;
            if answer == ENTER {
                answer = default.into();
            }
            valid(&answer).unwrap_or_else(|why| panic!("{answer:?} refused: {why}"));
            Ok(answer)
        }
        fn password(&self, prompt: &str, valid: Validate) -> Result<String> {
            self.input(prompt, "", "", valid)
        }
        fn confirm(&self, question: &str) -> Result<bool> {
            self.show(format!("? {question}"));
            Ok(self.answer(question)? == "yes")
        }
        fn outro(&self, text: &str) -> Result<()> {
            self.show(format!("└ {text}"));
            Ok(())
        }
        fn cancel(&self, text: &str) -> Result<()> {
            self.show(format!("■ {text}"));
            Ok(())
        }
    }
}
