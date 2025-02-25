use parse::{BitRange, Periph, Var, Reg};
use proc_macro::TokenStream;
use syn::{parse_macro_input, spanned::Spanned, Attribute, Ident};
use heck::*;
use proc_macro2::{Literal, Span, TokenStream as TokenStream2};
use quote::quote;

mod parse;

macro_rules! ident {
    ($format:expr, $ident:expr) => {
        Ident::new(&(format!($format, $ident)), $ident.span())
    };
}

struct IdentGen {
    counter: usize
}

impl IdentGen {
    pub fn new() -> IdentGen {
        IdentGen {
            counter: 0
        }
    }

    fn next(&mut self) -> Ident {
        let ident = Ident::new(&format!("_{}", self.counter), Span::call_site());
        self.counter += 1;
        ident
    }
}

fn reg_ident(reg: &Ident) -> Ident {
    Ident::new(&("Reg".to_string() + &reg.to_string().to_upper_camel_case()), reg.span())
}

#[proc_macro]
pub fn dev_csr(input: TokenStream) -> TokenStream {
    let Periph {
        addr_ty,
        word_ty,
        name,
        regs,
        vars,
        keywords,
        ..
    } = parse_macro_input!(input as Periph);

    let addr_trait = ident!("As{}Addr", name);
    let read_trait = ident!("Read{}", name);
    let write_trait = ident!("Write{}", name);

    let mut out = TokenStream2::new();

    let mut read_out = TokenStream2::new();
    let mut write_out = TokenStream2::new();

    // coloring on keywords
    for kw in keywords {
        let fake_kw = Ident::new("const", kw.span());
        out.extend(quote! {
            #fake_kw _: u8 = 0;
        });
    }

    for reg in regs {
        let reg_struct = reg_ident(&reg.reg);
        let Reg { addr, access, .. } = reg;
        let readable = access.readable;
        let writable = access.writable;

        out.extend(quote!{
            pub struct #reg_struct;
            impl #reg_struct {
                pub const ADDR: #addr_ty = #addr;
                pub const READABLE: bool = #readable;
                pub const WRITABLE: bool = #writable;
            }

            impl #addr_trait for #reg_struct {
                fn as_addr(&self) -> #addr_ty {
                    #addr
                }
            }
        });

        if readable {
            out.extend(quote! {
                impl ReadableAddr for #reg_struct {}
            });
        }

        if writable {
            out.extend(quote! {
                impl WritableAddr for #reg_struct {}
            });
        }
    }

    for (_, var) in &vars {
        let IoFn { read, write } = gen_io_fn(var);

        let mut name = var.parts[0].var.clone();
        name.set_span(Span::call_site());
        let mut set_var = ident!("set_{}", name);
        set_var.set_span(Span::call_site());
        let var_ty = &var.ty;
        let attr: Vec<&Attribute> = (&var.parts).iter().flat_map(|part| &part.attr).collect();

        if var.access.readable {
            if let Some(read) = read {
                read_out.extend(quote! {
                    #(#attr)*
                    fn #name(&mut self) -> impl core::future::Future<Output = core::result::Result<#var_ty, Self::Error>> {
                        #read
                    }
                })
            }
        }

        if var.access.writable {
            if let Some(write) = write {
                write_out.extend(quote! {
                    #(#attr)*
                    fn #set_var(&mut self, value: #word_ty) -> impl core::future::Future<Output = core::result::Result<(), Self::Error>> {
                        #write
                    }
                })
            }
        }
    }

    out.extend(quote! {
        pub trait #addr_trait {
            fn as_addr(&self) -> #addr_ty;
        }
        pub trait ReadableAddr: #addr_trait {}
        pub trait WritableAddr: #addr_trait {}

        pub trait ReadableValue<T> {
            fn from_value(value: T) -> Self;
        }
        pub trait WritableValue<T> {
            fn into_value(self) -> T;
        }

        /*
        pub struct Transmuted<T>;
        impl <T> ReadableValue for Transmuted<T> {
            fn from_value<T>(value: T) -> self {
                unsafe {
                    core::mem::transmute(value)
                }
            }
        }
        impl <T> WritableValue for Transmuted<T> {

        }*/

        impl #addr_trait for #addr_ty {
            fn as_addr(&self) -> #addr_ty {
                *self
            }
        }
        impl ReadableAddr for #addr_ty {}
        impl WritableAddr for #addr_ty {}

        pub trait #read_trait {
            type Error;

            fn read_contiguous_regs(
                &mut self,
                addr: impl ReadableAddr,
                out: &mut [#word_ty]
            ) -> impl core::future::Future<Output = core::result::Result<(), Self::Error>>;

            fn read_reg(
                &mut self,
                addr: impl ReadableAddr
            ) -> impl core::future::Future<Output = core::result::Result<#word_ty, Self::Error>> {
                async move {
                    let mut out = [0];
                    match self.read_contiguous_regs(addr, &mut out).await {
                        Ok(_) => Ok(out[0]),
                        Err(err) => Err(err)
                    }
                }
            }

            #read_out
        }

        pub trait #write_trait {
            type Error;

            /// Write consecutive words to the peripheral. Most chips have it so
            /// that when writing more than just one word, the next word goes into the
            /// next address.
            fn write_contiguous_regs(
                &mut self,
                addr: impl WritableAddr,
                values: &[#word_ty],
            ) -> impl core::future::Future<Output = core::result::Result<(), Self::Error>>;

            /// Write one word to the peripheral. Default calls `write_contiguous_regs` with length `1`.
            fn write_reg(
                &mut self,
                addr: impl WritableAddr,
                value: #word_ty
            ) -> impl core::future::Future<Output = core::result::Result<(), Self::Error>> {
                async move {
                    let words = [value];
                    self.write_contiguous_regs(addr, &words).await
                }
            }

            #write_out
        }
    });

    out.into()
}

struct IoFn {
    read: Option<TokenStream2>,
    write: Option<TokenStream2>
}
fn gen_io_fn(var: &Var) -> IoFn {
    let var_ty = &var.ty;
    let mut should_write = true;

    if var.parts.len() == 0 {
        panic!("Var with no parts???");
    }

    //let name = &var.parts[0].var;

    if var.parts.len() == 1 {
        let part = &var.parts[0];
        let reg = reg_ident(&part.reg);

        // Special case: 1-to-1 mapping
        if part.reg_range == BitRange::Entire && part.var_range == BitRange::Entire {
            return IoFn {
                read: Some(quote! {
                    self.read_reg(#reg)
                }),
                write: Some(quote! {
                    self.write_reg(#reg, value)
                })
            }
        }

        // Special case: boolean
        if let BitRange::Single(index) = part.reg_range {
            if part.var_range == BitRange::Entire {
                let index = Literal::usize_unsuffixed(index);

                return IoFn {
                    read: Some(quote! {
                        async move {
                            let word = self.read_reg(#reg).await?;
                            let word = (word >> #index) % 1;
                            unsafe {
                                Ok(core::mem::transmute(word))
                            }
                        }
                    }),
                    write: None,
                }
            }
        }
    }

    let mut gen = IdentGen::new();

    struct Contig {
        start: Ident,
        start_addr: usize,
        /// Ident that corresponds to each word.
        words: Vec<Ident>
    }

    let mut read_var_out = TokenStream2::new();
    let mut write_var_out = TokenStream2::new();

    let mut contigs: Vec<Contig> = vec![];

    let acc = gen.next();

    for part in &var.parts {
        let addr = part.reg_addr.base10_parse().unwrap();
        let ident = gen.next();
        
        let contig = match contigs.last() {
            Some(last) if last.start_addr + last.words.len() == addr => {
                let mut contig = contigs.pop().unwrap();
                contig.words.push(ident.clone());
                contig
            },
            _ => {
                Contig {
                    start: reg_ident(&part.reg),
                    start_addr: addr,
                    words: vec![ident.clone()]
                }
            }
        };

        match part.reg_range {
            BitRange::Entire => {
                let var_start = Literal::usize_unsuffixed(part.var_range.start().unwrap());
                let var_end = Literal::usize_unsuffixed(part.var_range.end().unwrap());

                read_var_out.extend(quote! {
                    #acc += (#ident as #var_ty) << #var_start;
                });

                write_var_out.extend(quote! {
                    let #ident = ((#acc % (1 << (#var_end + 1))) >> #var_start);
                });
            },
            _ => {
                let reg_start = part.reg_range.start().unwrap();
                let reg_end = part.reg_range.end().unwrap();
                let var_start = match part.var_range {
                    BitRange::Entire => 0,
                    range => range.start().unwrap()
                };

                let reg_start = Literal::usize_unsuffixed(reg_start);
                let reg_end = Literal::usize_unsuffixed(reg_end);
                let var_start = Literal::usize_unsuffixed(var_start);
                
                read_var_out.extend(quote! {
                    #acc += (((#ident % (1 << (#reg_end + 1))) as #var_ty) >> #reg_start) << #var_start;
                });

                // Don't write -- theres other stuff in the register that we would overwrite
                should_write = false;
                /*match part.var_range.end() {
                    Some(var_end) => {
                        write_var_out.extend(quote! {
                            let #ident = ((#acc % (#var_end + 1)) >> #var_start) << #reg_start;
                        })
                    },
                    None => {
                        write_var_out.extend(quote! {
                            let #ident = (#acc >> #var_start) << #reg_start;
                        })
                    }
                }       */         
            }
        }

        contigs.push(contig);
    }

    let mut read_begin = TokenStream2::new();
    let mut write_end = TokenStream2::new();

    for contig in contigs {
        if contig.words.len() == 1 {
            let ident = &contig.words[0];
            let reg = contig.start;

            read_begin.extend(quote! {
                let #ident = self.read_reg(#reg).await?;
            });

            write_end.extend(quote! {
                self.write_reg(#reg, #ident).await?;
            });
        } else {
            let words = &contig.words;
            let len = contig.words.len();
            let start = contig.start;

            read_begin.extend(quote! {
                let mut read = [0u8; #len];
                self.read_contiguous_regs(#start, &mut read).await?;
                let [#(#words),*] = read;
            });

            write_end.extend(quote! {
                self.write_contiguous_regs::<#len>(#start, &[#(#words),*]).await?;
            });
        }
    }

    return IoFn {
        read: Some(quote! {
            async move {
                #read_begin
                let mut #acc: #var_ty = 0;
                #read_var_out
                Ok(#acc)
            }
        }),
        write: if should_write {
            Some(quote! {
                async move {
                    let mut #acc = value;
                    #write_var_out
                    #write_end
                    Ok(())
                }
            })
        } else {
            None
        }
    }
}